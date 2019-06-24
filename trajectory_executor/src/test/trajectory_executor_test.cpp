/*
 * Copyright (C) 2018-2019 LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License") { you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

#include "test_utils.h"

TEST_F(TrajectoryExecutorTestSuite, test_message_no_crash) {
    waitForSubscribers(traj_pub, 1, 500);
    cav_msgs::TrajectoryPlan plan = buildSampleTraj();

    traj_pub.publish(plan);

    boost::shared_ptr<const cav_msgs::SystemAlert> msg = ros::topic::waitForMessage<cav_msgs::SystemAlert>("system_alert", ros::Duration(0.5));
    ASSERT_TRUE(!msg) << "Received system shutdown alert message from TrajectoryExecutor node.";
}

TEST_F(TrajectoryExecutorTestSuite, test_emit_traj) {
    waitForSubscribers(traj_pub, 1, 500);
    cav_msgs::TrajectoryPlan plan = buildSampleTraj();

    traj_pub.publish(plan);

    boost::shared_ptr<const cav_msgs::TrajectoryPlan> msg = ros::topic::waitForMessage<cav_msgs::TrajectoryPlan>("/carma/guidance/control_plugins/pure_pursuit/trajectory", ros::Duration(1.5));
    ASSERT_FALSE(!msg) << "Failed to receive trajectory execution message from TrajectoryExecutor node.";
} 

TEST_F(TrajectoryExecutorTestSuite, test_runover_shutdown) {
    waitForSubscribers(traj_pub, 1, 500);
    cav_msgs::TrajectoryPlan plan = buildSampleTraj();

    traj_pub.publish(plan);

    boost::shared_ptr<const cav_msgs::SystemAlert> msg = ros::topic::waitForMessage<cav_msgs::SystemAlert>("system_alert", ros::Duration(1.5));
    ASSERT_FALSE(!msg) << "Failed to receive system shutdown alert message from TrajectoryExecutor node.";
}

int main (int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "trajectory_executor_test");

    std::thread spinner([] {while (ros::ok()) ros::spin();});

    auto res = RUN_ALL_TESTS();

    ros::shutdown();

    return res;
}
