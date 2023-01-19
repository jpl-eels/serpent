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
        self.optimized_path = None
        self.registration = None
        self.optimized_pose_curr = None
        self.optimized_pose_prev = None

        self.optimized_path_sub = rospy.Subscriber(
            "/serpent/output/path", Path, self.optimized_path_cb
        )
        self.optimized_pose_sub = rospy.Subscriber(
            "/serpent/output/pose", PoseWithCovarianceStamped, self.optimized_pose_cb
        )
        self.registration_tf_sub = rospy.Subscriber(
            "/serpent/registration/transform",
            PoseWithCovarianceStamped,
            self.registration_cb,
        )
        self.metrics_pub = rospy.Publisher(
            "/serpent/metrics", DictionaryList, queue_size=10
        )
        rospy.loginfo("Initialized metrics node")

    def optimized_path_cb(self, msg):
        self.optimized_path = msg

    def optimized_pose_cb(self, msg):
        self.optimized_pose_prev = copy.deepcopy(self.optimized_pose_curr)
        self.optimized_pose_curr = msg

    def registration_cb(self, msg):
        self.registration = msg

    def add_metric(self, name, unit, value, metrics_message):
        metric = Dictionary()
        metric.name = name
        metric.unit = unit
        metric.value = value
        metrics_message.data.append(metric)
        return metrics_message

    def get_var_diag(self):
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

    def get_var_outlier(self):
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

    def get_rel_pose(self, pose1, pose2):
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
        dot = min(1, np.dot(q1, q2))
        angle = 2 * np.arccos(dot)
        return norm, angle

    def get_metrics(self):
        """ Computes and publishes metrics
        """
        metrics = DictionaryList()

        # registration covariance diagonal
        rx, ry, rz, tx, ty, tz = self.get_var_diag()
        metrics = self.add_metric(
            MetricDefines.REGISTRATION_VAR_RX, Dictionary.UNIT_NONE, rx, metrics
        )
        metrics = self.add_metric(
            MetricDefines.REGISTRATION_VAR_RY, Dictionary.UNIT_NONE, ry, metrics
        )
        metrics = self.add_metric(
            MetricDefines.REGISTRATION_VAR_RZ, Dictionary.UNIT_NONE, rz, metrics
        )
        metrics = self.add_metric(
            MetricDefines.REGISTRATION_VAR_TX, Dictionary.UNIT_NONE, tx, metrics
        )
        metrics = self.add_metric(
            MetricDefines.REGISTRATION_VAR_TY, Dictionary.UNIT_NONE, ty, metrics
        )
        metrics = self.add_metric(
            MetricDefines.REGISTRATION_VAR_TZ, Dictionary.UNIT_NONE, tz, metrics
        )

        # covariance outliers
        covariance_outlier_max, covariance_outlier_idx = self.get_var_outlier()
        metrics = self.add_metric(
            MetricDefines.VARIANCE_OUTLIER_MAX,
            Dictionary.UNIT_NONE,
            covariance_outlier_max,
            metrics,
        )
        metrics = self.add_metric(
            MetricDefines.VARIANCE_OUTLIER_IDX,
            Dictionary.UNIT_NONE,
            covariance_outlier_idx,
            metrics,
        )

        # optimized tf diff (use last and second to last poses in optimized pose seq)
        if self.optimized_path is not None:
            opt_tf_diff_norm, opt_tf_diff_angl = self.get_rel_pose(
                self.optimized_path.poses[-1].pose, self.optimized_path.poses[-2].pose
            )
        else:
            opt_tf_diff_norm, opt_tf_diff_angl = -1.0, -1.0
        metrics = self.add_metric(
            MetricDefines.CONSISTENT_RELATIVE_POSE_UPDATE_NORM,
            Dictionary.UNIT_NONE,
            opt_tf_diff_norm,
            metrics,
        )
        metrics = self.add_metric(
            MetricDefines.CONSISTENT_RELATIVE_POSE_UPDATE_ANGL,
            Dictionary.UNIT_NONE,
            opt_tf_diff_angl,
            metrics,
        )

        # non optimized tf diff (use current and previous optimized poses)
        if (
            self.optimized_pose_curr is not None
            and self.optimized_pose_prev is not None
        ):
            non_opt_tf_diff_norm, non_opt_tf_diff_angl = self.get_rel_pose(
                self.optimized_pose_curr.pose.pose, self.optimized_pose_prev.pose.pose
            )
        else:
            non_opt_tf_diff_norm, non_opt_tf_diff_angl = -1.0, -1.0
        metrics = self.add_metric(
            MetricDefines.INSTANT_OPTIMIZED_RELATIVE_POSE_NORM,
            Dictionary.UNIT_NONE,
            non_opt_tf_diff_norm,
            metrics,
        )
        metrics = self.add_metric(
            MetricDefines.INSTANT_OPTIMIZED_RELATIVE_POSE_ANGL,
            Dictionary.UNIT_NONE,
            non_opt_tf_diff_angl,
            metrics,
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
        metrics = metrics_calculator.get_metrics()
        metrics_calculator.publish_metrics(metrics)
        rate.sleep()


if __name__ == "__main__":
    main()
