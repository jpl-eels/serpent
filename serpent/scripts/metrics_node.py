#!/usr/bin/env python

import rospy
from perception_msgs.msg import MetricDefines
from infrastructure_msgs.msg import Dictionary, DictionaryList
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Path
import numpy as np
import copy


class MetricsCalculator(object):
    def __init__(self):
        self.optimized_pose_seq = None
        self.registration = None
        self.optimized_pose_curr = None
        self.optimized_pose_prev = None

        self.optimized_pose_seq_sub = rospy.Subscriber(
            "/serpent/output/path", Path, self.optimized_pose_seq_cb
        )
        self.optimized_pose_sub = rospy.Subscriber(
            "/serpent/output/pose", PoseWithCovarianceStamped, self.optimized_pose_cb
        )
        self.registration_sub = rospy.Subscriber(
            "/serpent/registration/transform",
            PoseWithCovarianceStamped,
            self.registration_cb,
        )
        self.metrics_pub = rospy.Publisher(
            "/serpent/metrics", DictionaryList, queue_size=10
        )
        rospy.loginfo("Initialized metrics node")

    def optimized_pose_seq_cb(self, msg):
        self.optimized_pose_seq = msg

    def optimized_pose_cb(self, msg):
        # copy current pose to previous pose
        self.optimized_pose_prev = copy.deepcopy(self.optimized_pose_curr)
        self.optimized_pose_curr = msg

    def registration_cb(self, msg):
        self.registration = msg

    def compile_metric(self, name, unit, value):
        metric = Dictionary()
        metric.name = name
        metric.unit = unit
        metric.value = value
        return metric

    def get_registration_diag(self):
        """ gets diagonal elements of covariance matrix
        """
        if self.registration is None:
            return -1.0, -1.0, -1.0, -1.0, -1.0, -1.0
        rx = self.registration.pose.covariance[0]
        ry = self.registration.pose.covariance[7]
        rz = self.registration.pose.covariance[14]
        tx = self.registration.pose.covariance[21]
        ty = self.registration.pose.covariance[28]
        tz = self.registration.pose.covariance[35]
        return rx, ry, rz, tx, ty, tz

    def get_outlier(self):
        """ looks at covariance matrix and returns the maximum value and its index
        """
        if self.optimized_pose_curr is None:
            return -1.0, -1.0
        covariance_outlier_max = 0.0
        covariance_outlier_idx = -1
        # look through optimized_pose_curr and check what the maximum covariance value is
        for i in range(0, len(self.optimized_pose_curr.pose.covariance)):
            if self.optimized_pose_curr.pose.covariance[i] > covariance_outlier_max:
                covariance_outlier_max = self.optimized_pose_curr.pose.covariance[i]
                covariance_outlier_idx = i
        return covariance_outlier_max, covariance_outlier_idx

    def get_tf_diff(self, pose1, pose2):
        """ returns norm of pose1.position - pose2.position and angle between pose1.orientation and pose2.orientation 
        """
        # norm
        diff = np.array(
            [pose1.position.x, pose1.position.y, pose1.position.z]
        ) - np.array([pose2.position.x, pose2.position.y, pose2.position.z])
        norm = np.linalg.norm(diff)

        # angle
        q1 = np.array(
            [
                pose1.orientation.x,
                pose1.orientation.y,
                pose1.orientation.z,
                pose1.orientation.w,
            ]
        )
        q2 = np.array(
            [
                pose2.orientation.x,
                pose2.orientation.y,
                pose2.orientation.z,
                pose2.orientation.w,
            ]
        )
        dot = np.dot(q1, q2)
        angle = abs(np.arccos(2 * dot * dot - 1))

        return norm, angle

    def calculate_metrics(self):
        """ Computes and publishes metrics
        """
        metrics = DictionaryList()

        # registration covariance diagonal
        rx, ry, rz, tx, ty, tz = self.get_registration_diag()
        metrics.data.append(
            self.compile_metric(MetricDefines.REGISTRATION_RX, Dictionary.UNIT_NONE, rx)
        )
        metrics.data.append(
            self.compile_metric(MetricDefines.REGISTRATION_RY, Dictionary.UNIT_NONE, ry)
        )
        metrics.data.append(
            self.compile_metric(MetricDefines.REGISTRATION_RZ, Dictionary.UNIT_NONE, rz)
        )
        metrics.data.append(
            self.compile_metric(MetricDefines.REGISTRATION_TX, Dictionary.UNIT_NONE, tx)
        )
        metrics.data.append(
            self.compile_metric(MetricDefines.REGISTRATION_TY, Dictionary.UNIT_NONE, ty)
        )
        metrics.data.append(
            self.compile_metric(MetricDefines.REGISTRATION_TZ, Dictionary.UNIT_NONE, tz)
        )

        # covariance outliers
        covariance_outlier_max, covariance_outlier_idx = self.get_outlier()
        metrics.data.append(
            self.compile_metric(
                MetricDefines.COVARIANCE_OUTLIER_MAX,
                Dictionary.UNIT_NONE,
                covariance_outlier_max,
            )
        )
        metrics.data.append(
            self.compile_metric(
                MetricDefines.COVARIANCE_OUTLIER_IDX,
                Dictionary.UNIT_NONE,
                covariance_outlier_idx,
            )
        )

        # optimized tf diff (use last and second to last poses in optimized pose seq)
        if self.optimized_pose_seq is not None:
            opt_tf_diff_norm, opt_tf_diff_angl = self.get_tf_diff(
                self.optimized_pose_seq.poses[-1].pose,
                self.optimized_pose_seq.poses[-2].pose,
            )
        else:
            opt_tf_diff_norm, opt_tf_diff_angl = -1.0, -1.0
        metrics.data.append(
            self.compile_metric(
                MetricDefines.OPTIMIZED_TRANSFORM_UPDATE_NORM,
                Dictionary.UNIT_NONE,
                opt_tf_diff_norm,
            )
        )
        metrics.data.append(
            self.compile_metric(
                MetricDefines.OPTIMIZED_TRANSFORM_UPDATE_ANGL,
                Dictionary.UNIT_NONE,
                opt_tf_diff_angl,
            )
        )

        # non optimized tf diff (use current and previous optimized poses)
        if (
            self.optimized_pose_curr is not None
            and self.optimized_pose_prev is not None
        ):
            non_opt_tf_diff_norm, non_opt_tf_diff_angl = self.get_tf_diff(
                self.optimized_pose_curr.pose.pose, self.optimized_pose_prev.pose.pose
            )
        else:
            non_opt_tf_diff_norm, non_opt_tf_diff_angl = -1.0, -1.0
        metrics.data.append(
            self.compile_metric(
                MetricDefines.NON_OPTIMIZED_TRANSFORM_UPDATE_NORM,
                Dictionary.UNIT_NONE,
                non_opt_tf_diff_norm,
            )
        )
        metrics.data.append(
            self.compile_metric(
                MetricDefines.NON_OPTIMIZED_TRANSFORM_UPDATE_ANGL,
                Dictionary.UNIT_NONE,
                non_opt_tf_diff_angl,
            )
        )

        return metrics

    def publish_metrics(self, metrics):
        """ Publishes metrics
        """
        self.metrics_pub.publish(metrics)


def main():
    rospy.init_node("metrics_node")
    metrics_calculator = MetricsCalculator()
    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        metrics = metrics_calculator.calculate_metrics()
        metrics_calculator.publish_metrics(metrics)
        rate.sleep()


if __name__ == "__main__":
    main()
