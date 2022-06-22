#include "serpent/graph_manager.hpp"
#include "test/read_data.hpp"
#include "test/test_data.hpp"
#include "test/test_utils.hpp"
#include <gtest/gtest.h>
#include <gtsam/inference/Symbol.h>

#define DOUBLE_PRECISION (1.0e-12)

using gtsam::symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
using gtsam::symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)

TEST(graph_manager, imu_bias_set_get_values) {
    serpent::GraphManager gm;
    const int size{5};
    for (int i = 0; i < size; ++i) {
        gm.set_imu_bias(i, test_imu_bias(i));
    }
    gtsam::Values v = gm.values(0, size - 1);
    EXPECT_EQ(v.size(), size);
    for (int i = 0; i < size; ++i) {
        const gtsam::imuBias::ConstantBias imu_bias = test_imu_bias(i);
        EXPECT_TRUE(gm.imu_bias(i).vector().isApprox(imu_bias.vector()));
        const gtsam::Key key = B(i);
        EXPECT_TRUE(v.exists(key));
        EXPECT_TRUE(v.at<gtsam::imuBias::ConstantBias>(key).vector().isApprox(imu_bias.vector()));
    }
}

TEST(graph_manager, imu_biases_set_get_values_named) {
    serpent::GraphManager gm;
    gm.set_named_key("bias");
    gm.set_named_key("bias_check");
    const int size{5};
    for (int i = 0; i < size; ++i) {
        gm.set_imu_bias("bias", test_imu_bias(gm.key("bias")));
        gm.increment("bias");
    }
    gtsam::Values v = gm.values(0, size - 1);
    EXPECT_EQ(v.size(), size);
    for (int i = 0; i < size; ++i) {
        const gtsam::imuBias::ConstantBias imu_bias = test_imu_bias(gm.key("bias_check"));
        EXPECT_TRUE(gm.imu_bias("bias_check").vector().isApprox(imu_bias.vector()));
        const gtsam::Key key = B(gm.key("bias_check"));
        EXPECT_TRUE(v.exists(key));
        EXPECT_TRUE(v.at<gtsam::imuBias::ConstantBias>(key).vector().isApprox(imu_bias.vector()));
        gm.increment("bias_check");
    }
}

TEST(graph_manager, navstate_set_get_values) {
    serpent::GraphManager gm;
    const int size{5};
    for (int i = 0; i < size; ++i) {
        gm.set_navstate(i, test_navstate(i));
    }
    gtsam::Values v = gm.values(0, size - 1);
    EXPECT_EQ(v.size(), 2*size);
    for (int i = 0; i < size; ++i) {
        const gtsam::NavState navstate = test_navstate(i);
        EXPECT_TRUE(gm.navstate(i).pose().matrix().isApprox(navstate.pose().matrix()));
        EXPECT_TRUE(gm.navstate(i).velocity().isApprox(navstate.velocity()));
        const gtsam::Key pose_key = X(i);
        EXPECT_TRUE(v.exists(pose_key));
        EXPECT_TRUE(v.at<gtsam::Pose3>(pose_key).matrix().isApprox(navstate.pose().matrix()));
        const gtsam::Key velocity_key = V(i);
        EXPECT_TRUE(v.exists(velocity_key));
        EXPECT_TRUE(v.at<gtsam::Velocity3>(velocity_key).isApprox(navstate.velocity()));
    }
}

TEST(graph_manager, poses_set_get_values) {
    serpent::GraphManager gm;
    const int size{5};
    for (int i = 0; i < size; ++i) {
        gm.set_pose(i, test_pose(i));
    }
    gtsam::Values v = gm.values(0, size - 1);
    EXPECT_EQ(v.size(), size);
    for (int i = 0; i < size; ++i) {
        const gtsam::Pose3 pose = test_pose(i);
        EXPECT_TRUE(gm.pose(i).matrix().isApprox(pose.matrix()));
        const gtsam::Key key = X(i);
        EXPECT_TRUE(v.exists(key));
        EXPECT_TRUE(v.at<gtsam::Pose3>(key).matrix().isApprox(pose.matrix()));
    }
}

TEST(graph_manager, poses_set_get_values_named) {
    serpent::GraphManager gm;
    gm.set_named_key("pose");
    gm.set_named_key("pose_check");
    const int size{5};
    for (int i = 0; i < size; ++i) {
        gm.set_pose("pose", test_pose(gm.key("pose")));
        gm.increment("pose");
    }
    gtsam::Values v = gm.values(0, size - 1);
    EXPECT_EQ(v.size(), size);
    for (int i = 0; i < size; ++i) {
        const gtsam::Pose3 pose = test_pose(gm.key("pose_check"));
        EXPECT_TRUE(gm.pose("pose_check").matrix().isApprox(pose.matrix()));
        const gtsam::Key key = X(gm.key("pose_check"));
        EXPECT_TRUE(v.exists(key));
        EXPECT_TRUE(v.at<gtsam::Pose3>(key).matrix().isApprox(pose.matrix()));
        gm.increment("pose_check");
    }
}

TEST(graph_manager, velocities_set_get_values) {
    serpent::GraphManager gm;
    const int size{5};
    for (int i = 0; i < size; ++i) {
        gm.set_velocity(i, test_velocity(i));
    }
    gtsam::Values v = gm.values(0, size - 1);
    EXPECT_EQ(v.size(), size);
    for (int i = 0; i < size; ++i) {
        const gtsam::Velocity3 vel = test_velocity(i);
        EXPECT_TRUE(gm.velocity(i).isApprox(vel));
        const gtsam::Key key = V(i);
        EXPECT_TRUE(v.exists(key));
        EXPECT_TRUE(v.at<gtsam::Velocity3>(key).isApprox(vel));
    }
}

TEST(graph_manager, velocities_set_get_values_named) {
    serpent::GraphManager gm;
    gm.set_named_key("vel");
    gm.set_named_key("vel_check");
    const int size{5};
    for (int i = 0; i < size; ++i) {
        gm.set_velocity("vel", test_velocity(gm.key("vel")));
        gm.increment("vel");
    }
    gtsam::Values v = gm.values(0, size - 1);
    EXPECT_EQ(v.size(), size);
    for (int i = 0; i < size; ++i) {
        const gtsam::Velocity3 vel = test_velocity(gm.key("vel_check"));
        EXPECT_TRUE(gm.velocity("vel_check").isApprox(vel));
        const gtsam::Key key = V(gm.key("vel_check"));
        EXPECT_TRUE(v.exists(key));
        EXPECT_TRUE(v.at<gtsam::Velocity3>(key).isApprox(vel));
        gm.increment("vel_check");
    }
}

TEST(graph_manager, timestamps_set_get) {
    serpent::GraphManager gm;
    const int size{5};
    for (int i = 0; i < size; ++i) {
        gm.set_timestamp(i, test_timestamp(i));
    }
    for (int i = 0; i < size; ++i) {
        EXPECT_EQ(gm.timestamp(i), test_timestamp(i));
    }
}

TEST(graph_manager, timestamps_set_get_named) {
    serpent::GraphManager gm;
    gm.set_named_key("ts");
    gm.set_named_key("ts_check");
    const int size{5};
    for (int i = 0; i < size; ++i) {
        gm.set_timestamp("ts", test_timestamp(i));
        gm.increment("ts");
    }
    for (int i = 0; i < size; ++i) {
        EXPECT_EQ(gm.timestamp("ts_check"), test_timestamp(i));
        gm.increment("ts_check");
    }
}

TEST(graph_manager, combined_imu_set_get_factor) {
    // Preintegration
    gtsam::PreintegratedCombinedMeasurements preintegrated_imu = test_preintegrated_measurements();
    auto imu_measurements = from_ros(read_imu(std::string(DATA_DIR) + "/imu_up_10msg.bag", "/imu"));
    for (std::size_t i = 1; i < imu_measurements.size(); ++i) {
        const auto& previous = imu_measurements.at(i - 1);
        const auto& current = imu_measurements.at(i);
        const double dt = (current.timestamp - previous.timestamp).toSec();
        EXPECT_GT(dt, 0.0);
        preintegrated_imu.integrateMeasurement(current.linear_acceleration, current.angular_velocity, dt);
    }

    // Create factor
    serpent::GraphManager gm;
    gm.create_combined_imu_factor(1, preintegrated_imu);
    gtsam::NonlinearFactorGraph factors = gm.factors(0, 1);
    EXPECT_EQ(factors.size(), 1);
    EXPECT_EQ(factors.keys().size(), 6);
}

TEST(graph_manager, between_pose_set_get_factor) {
    serpent::GraphManager gm;
    gm.create_between_pose_factor(1, test_pose(0), test_pose_noise_model(0));
    gtsam::NonlinearFactorGraph factors = gm.factors(0, 1);
    EXPECT_EQ(factors.size(), 1);
    EXPECT_EQ(factors.keys().size(), 2);
}

TEST(graph_manager, between_pose_set_get_factors) {
    serpent::GraphManager gm;
    const int size{5};
    for (int i = 0; i < size; ++i) {
        gm.create_between_pose_factor(i + 1, test_pose(i), test_pose_noise_model(i));
    }
    gtsam::NonlinearFactorGraph factors = gm.factors(0, size);
    EXPECT_EQ(factors.size(), size);
    EXPECT_EQ(factors.keys().size(), size + 1);
}

TEST(graph_manager, prior_imu_bias_set_get_factor) {
    serpent::GraphManager gm;
    gm.create_prior_imu_bias_factor(0, test_imu_bias(0), test_imu_bias_noise_model(0));
    gtsam::NonlinearFactorGraph factors = gm.factors(0, 0);
    EXPECT_EQ(factors.size(), 1);
    EXPECT_EQ(factors.keys().size(), 1);
}

TEST(graph_manager, prior_pose_set_get_factor) {
    serpent::GraphManager gm;
    gm.create_prior_pose_factor(0, test_pose(0), test_pose_noise_model(0));
    gtsam::NonlinearFactorGraph factors = gm.factors(0, 0);
    EXPECT_EQ(factors.size(), 1);
    EXPECT_EQ(factors.keys().size(), 1);
}

TEST(graph_manager, prior_velocity_set_get_factor) {
    serpent::GraphManager gm;
    gm.create_prior_velocity_factor(0, test_velocity(0), test_velocity_noise_model(0));
    gtsam::NonlinearFactorGraph factors = gm.factors(0, 0);
    EXPECT_EQ(factors.size(), 1);
    EXPECT_EQ(factors.keys().size(), 1);
}

TEST(graph_manager, combined_imu_with_priors_optimise) {
    serpent::ISAM2GraphManager gm{test_isam2_params()};
    
    // Prior values
    gm.set_timestamp(0, ros::Time(0));
    gm.set_imu_bias(0, gtsam::imuBias::ConstantBias{});
    gm.set_pose(0, gtsam::Pose3{});
    gm.set_velocity(0, gtsam::Velocity3{});

    // Priors
    gm.create_prior_imu_bias_factor(0, gm.imu_bias(0), test_imu_bias_noise_model(0));
    gm.create_prior_pose_factor(0, gm.pose(0), test_pose_noise_model(0));
    gm.create_prior_velocity_factor(0, gm.velocity(0), test_velocity_noise_model(0));
    
    // Preintegration
    gtsam::PreintegratedCombinedMeasurements preintegrated_imu = test_preintegrated_measurements();
    auto imu_measurements = from_ros(read_imu(std::string(DATA_DIR) + "/imu_up_10msg.bag", "/imu"));
    for (std::size_t i = 1; i < imu_measurements.size(); ++i) {
        const auto& previous = imu_measurements.at(i - 1);
        const auto& current = imu_measurements.at(i);
        const double dt = (current.timestamp - previous.timestamp).toSec();
        EXPECT_GT(dt, 0.0);
        preintegrated_imu.integrateMeasurement(current.linear_acceleration, current.angular_velocity, dt);
    }

    // Predict and update
    gm.set_timestamp(1, imu_measurements.back().timestamp);
    gm.set_imu_bias(1, gm.imu_bias(0));
    gm.set_navstate(1, preintegrated_imu.predict(gm.navstate(0), gm.imu_bias(0)));

    // Combined IMU factor
    gm.create_combined_imu_factor(1, preintegrated_imu);

    // Check values
    const gtsam::Values v = gm.values(0, 1);
    EXPECT_EQ(v.size(), 6);

    // Check factors
    const gtsam::NonlinearFactorGraph f = gm.factors(0, 1);
    EXPECT_EQ(f.size(), 4);

    // Optimise
    const gtsam::ISAM2Result result = gm.optimise(1);
    EXPECT_NEAR(*result.errorBefore, 0.0, DOUBLE_PRECISION);
    EXPECT_NEAR(*result.errorAfter, 0.0, DOUBLE_PRECISION);
    EXPECT_EQ(result.getVariablesReeliminated(), 6);
    EXPECT_EQ(result.getVariablesRelinearized(), 0);
}

TEST(graph_manager, update_from_values) {
    serpent::GraphManager gm;
    gtsam::Values values;
    // Initialise gm and values differently
    const int size{5};
    for (int i = 0; i < size - 1; ++i) {
        values.insert(B(i), test_imu_bias(i));
        gm.set_imu_bias(i, test_imu_bias(i + size));
        values.insert(X(i), test_pose(i));
        gm.set_pose(i, test_pose(i + size));
        values.insert(V(i), test_velocity(i));
        gm.set_velocity(i, test_velocity(i + size));
    }

    // Give gm an extra
    gm.set_imu_bias(size - 1, test_imu_bias(size - 1));
    gm.set_pose(size - 1, test_pose(size - 1));
    gm.set_velocity(size - 1, test_velocity(size - 1));

    // Load from values
    gm.update_from_values(values);
    
    // Check values
    for (int i = 0; i < size; ++i) {
        EXPECT_TRUE(gm.imu_bias(i).vector().isApprox(test_imu_bias(i).vector()));
        EXPECT_TRUE(gm.pose(i).matrix().isApprox(test_pose(i).matrix()));
        EXPECT_TRUE(gm.velocity(i).isApprox(test_velocity(i)));
    }
}
