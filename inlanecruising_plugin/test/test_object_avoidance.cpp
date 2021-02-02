/*
 * Copyright (C) 2019-2020 LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
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

#include <inlanecruising_plugin/object_avoidance.h>
#include <inlanecruising_plugin/inlanecruising_plugin.h>
#include <trajectory_utils/quintic_coefficient_calculator.h>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <carma_wm/CARMAWorldModel.h>
#include <math.h>
#include <tf/LinearMath/Vector3.h>

using namespace inlanecruising_plugin;

TEST(InLaneCruisingPluginTest, test_polynomial_calc)
{

    std::vector<double> coeff;

    coeff.push_back(2.0);
    coeff.push_back(2.0);
    coeff.push_back(2.0);
    coeff.push_back(2.0);
    coeff.push_back(2.0);    
    coeff.push_back(2.0);

    object_avoidance::ObjectAvoidance obj;    

    double result = obj.polynomial_calc(coeff, 0);
    EXPECT_EQ(2, result);

    result = obj.polynomial_calc(coeff, 1);
    EXPECT_EQ(12, result);

    result = obj.polynomial_calc(coeff, 2);
    EXPECT_EQ(126, result);

    result = obj.polynomial_calc(coeff, 3);
    EXPECT_EQ(728, result);
}

TEST(InLaneCruisingPluginTest, test_polynomial_calc_d)
{

    std::vector<double> coeff;

    coeff.push_back(2.0);
    coeff.push_back(2.0);
    coeff.push_back(2.0);
    coeff.push_back(2.0);
    coeff.push_back(2.0);    
    coeff.push_back(2.0);

    object_avoidance::ObjectAvoidance obj;    

    double result = obj.polynomial_calc_d(coeff, 0);
    EXPECT_EQ(2, result);

    result = obj.polynomial_calc_d(coeff, 1);
    EXPECT_EQ(30, result);

    result = obj.polynomial_calc_d(coeff, 2);
    EXPECT_EQ(258, result);

    result = obj.polynomial_calc_d(coeff, 3);
    EXPECT_EQ(1094, result);
}

TEST(InLaneCruisingPluginTest, MaxTrajectorySpeed)
{

    std::vector<cav_msgs::TrajectoryPlanPoint> trajectory_points;

    ros::Time startTime(1.0);

    cav_msgs::TrajectoryPlanPoint point_1;
    point_1.x = 0.0;
    point_1.y = 0.0;
    point_1.target_time = startTime;
    point_1.lane_id = "1";
    trajectory_points.push_back(point_1);

    cav_msgs::TrajectoryPlanPoint point_2;
    point_2.x = 5.0;
    point_2.y = 0.0;
    point_2.target_time = startTime + ros::Duration(1);
    point_2.lane_id = "1";
    trajectory_points.push_back(point_2);

    cav_msgs::TrajectoryPlanPoint point_3;
    point_3.x = 10.0;
    point_3.y = 0.0;
    point_3.target_time = startTime + ros::Duration(2);
    point_3.lane_id = "1";
    trajectory_points.push_back(point_3);

    cav_msgs::TrajectoryPlanPoint point_4;
    point_4.x = 15.0;
    point_4.y = 0.0;
    point_4.target_time = startTime + ros::Duration(3);
    point_4.lane_id = "1";
    trajectory_points.push_back(point_4);

    cav_msgs::TrajectoryPlanPoint point_5;
    point_5.x = 20.0;
    point_5.y = 0.0;
    point_5.target_time = startTime + ros::Duration(4);
    point_5.lane_id = "1";
    trajectory_points.push_back(point_5);

    cav_msgs::TrajectoryPlanPoint point_6;
    point_6.x = 25.0;
    point_6.y = 0.0;
    point_6.target_time = startTime + ros::Duration(5);
    point_6.lane_id = "1";
    trajectory_points.push_back(point_6);


    cav_msgs::TrajectoryPlanPoint point_7;
    point_7.x = 40.0;
    point_7.y = 0.0;
    point_7.target_time = startTime + ros::Duration(6);
    point_7.lane_id = "1";
    trajectory_points.push_back(point_7);

    object_avoidance::ObjectAvoidance obj;
    double result = obj.max_trajectory_speed(trajectory_points);
    EXPECT_EQ(5, result);

}

TEST(InLaneCruisingPluginTest, get_relative_downtracks_test)
{

    std::vector<cav_msgs::TrajectoryPlanPoint> trajectory_points;

    ros::Time startTime(1.0);

    cav_msgs::TrajectoryPlanPoint point_1;
    point_1.x = 0.0;
    point_1.y = 0.0;
    point_1.target_time = startTime;
    point_1.lane_id = "1";
    trajectory_points.push_back(point_1);

    cav_msgs::TrajectoryPlanPoint point_2;
    point_2.x = 5.0;
    point_2.y = 0.0;
    point_2.target_time = startTime + ros::Duration(1);
    point_2.lane_id = "1";
    trajectory_points.push_back(point_2);

    cav_msgs::TrajectoryPlanPoint point_3;
    point_3.x = 10.0;
    point_3.y = 0.0;
    point_3.target_time = startTime + ros::Duration(2);
    point_3.lane_id = "1";
    trajectory_points.push_back(point_3);

    cav_msgs::TrajectoryPlanPoint point_4;
    point_4.x = 15.0;
    point_4.y = 0.0;
    point_4.target_time = startTime + ros::Duration(3);
    point_4.lane_id = "1";
    trajectory_points.push_back(point_4);

    cav_msgs::TrajectoryPlanPoint point_5;
    point_5.x = 20.0;
    point_5.y = 0.0;
    point_5.target_time = startTime + ros::Duration(4);
    point_5.lane_id = "1";
    trajectory_points.push_back(point_5);

    cav_msgs::TrajectoryPlanPoint point_6;
    point_6.x = 25.0;
    point_6.y = 0.0;
    point_6.target_time = startTime + ros::Duration(5);
    point_6.lane_id = "1";
    trajectory_points.push_back(point_6);


    cav_msgs::TrajectoryPlanPoint point_7;
    point_7.x = 40.0;
    point_7.y = 0.0;
    point_7.target_time = startTime + ros::Duration(6);
    point_7.lane_id = "1";
    trajectory_points.push_back(point_7);

    cav_msgs::TrajectoryPlan tp;
    tp.trajectory_points = trajectory_points;

    object_avoidance::ObjectAvoidance obj;
    std::vector<double> result = obj.get_relative_downtracks(tp);
    EXPECT_EQ(0, result[0]);
    EXPECT_EQ(5, result[1]);
    EXPECT_EQ(5, result[2]);
    EXPECT_EQ(5, result[3]);
    EXPECT_EQ(5, result[4]);
    EXPECT_EQ(5, result[5]);
    EXPECT_EQ(15, result[6]);
    

}

TEST(InLaneCruisingPluginTest, test_update_traj)
{

    std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();

    cav_msgs::RoadwayObstacleList rwol;
    cav_msgs::TrajectoryPlan tp;


    
    geometry_msgs::Vector3 size;
    size.x = 4;
    size.y = 2;
    size.z = 1;

    ros::Time startTime(1.0);

    cav_msgs::TrajectoryPlanPoint trajectory_point_1;
    cav_msgs::TrajectoryPlanPoint trajectory_point_2;
    cav_msgs::TrajectoryPlanPoint trajectory_point_3;
    cav_msgs::TrajectoryPlanPoint trajectory_point_4;
    cav_msgs::TrajectoryPlanPoint trajectory_point_5;
    cav_msgs::TrajectoryPlanPoint trajectory_point_6;
    cav_msgs::TrajectoryPlanPoint trajectory_point_7;

    trajectory_point_1.x = 1.0;
    trajectory_point_1.y = 1.0;
    trajectory_point_1.target_time = ros::Time(0);

    trajectory_point_2.x = 10.0;
    trajectory_point_2.y = 20.0;
    trajectory_point_2.target_time = ros::Time(0, 1);

    trajectory_point_3.x = 10.0;
    trajectory_point_3.y = 30.0;
    trajectory_point_3.target_time = ros::Time(0, 2);

    trajectory_point_4.x = 10.0;
    trajectory_point_4.y = 40.0;
    trajectory_point_4.target_time = ros::Time(0, 3);

    trajectory_point_5.x = 10.0;
    trajectory_point_5.y = 50.0;
    trajectory_point_5.target_time = ros::Time(0, 4);

    trajectory_point_6.x = 10.0;
    trajectory_point_6.y = 60.0;
    trajectory_point_6.target_time = ros::Time(0, 5);

    trajectory_point_7.x = 10.0;
    trajectory_point_7.y = 70.0;
    trajectory_point_7.target_time = ros::Time(0, 6);

    tp.trajectory_points = {trajectory_point_1, trajectory_point_2, trajectory_point_3, trajectory_point_4, trajectory_point_5, trajectory_point_6, trajectory_point_7};

    cav_msgs::RoadwayObstacle rwo_1;

    tf2::Quaternion tf_orientation;
    tf_orientation.setRPY(0, 0, 1.5708);

    rwo_1.object.pose.pose.position.x = 60;
    rwo_1.object.pose.pose.position.y = 50;
    rwo_1.object.pose.pose.position.z = 0;

    rwo_1.object.pose.pose.orientation.x = tf_orientation.getX();
    rwo_1.object.pose.pose.orientation.y = tf_orientation.getY();
    rwo_1.object.pose.pose.orientation.z = tf_orientation.getZ();
    rwo_1.object.pose.pose.orientation.w = tf_orientation.getW();

    rwo_1.object.size.x = 1;
    rwo_1.object.size.y = 1;
    rwo_1.object.size.z = 1;

    cav_msgs::PredictedState ps_1;
    ps_1.header.stamp.nsec = 1000;

    ps_1.predicted_position.position.x = 10;
    ps_1.predicted_position.position.y = 10;
    ps_1.predicted_position.position.z = 0;

    ps_1.predicted_position.orientation.x = tf_orientation.getX();
    ps_1.predicted_position.orientation.y = tf_orientation.getY();
    ps_1.predicted_position.orientation.z = tf_orientation.getZ();
    ps_1.predicted_position.orientation.w = tf_orientation.getW();

    cav_msgs::PredictedState ps_2;
    ps_2.header.stamp.nsec = 2000;

    ps_2.predicted_position.position.x = 10;
    ps_2.predicted_position.position.y = 20;
    ps_2.predicted_position.position.z = 0;

    ps_2.predicted_position.orientation.x = tf_orientation.getX();
    ps_2.predicted_position.orientation.y = tf_orientation.getY();
    ps_2.predicted_position.orientation.z = tf_orientation.getZ();
    ps_2.predicted_position.orientation.w = tf_orientation.getW();

    cav_msgs::PredictedState ps_3;
    ps_3.header.stamp.nsec = 3000;

    ps_3.predicted_position.position.x = 10;
    ps_3.predicted_position.position.y = 30;
    ps_3.predicted_position.position.z = 0;

    ps_3.predicted_position.orientation.x = tf_orientation.getX();
    ps_3.predicted_position.orientation.y = tf_orientation.getY();
    ps_3.predicted_position.orientation.z = tf_orientation.getZ();
    ps_3.predicted_position.orientation.w = tf_orientation.getW();

    rwo_1.object.predictions = {ps_1,ps_2,ps_3};
    rwo_1.object.velocity.twist.linear.x = 5.0;

    rwol.roadway_obstacles = {rwo_1};


    std::vector<cav_msgs::RoadwayObstacle> rw_objs;

    rw_objs.push_back(rwo_1);

    wm->setRoadwayObjects(rw_objs);

    object_avoidance::ObjectAvoidance obj;    

    obj.host_vehicle_size = size;

    cav_msgs::TrajectoryPlan tp_new = obj.update_traj_for_object(tp, wm, 10.0);

    for (size_t i = 1; i < tp_new.trajectory_points.size(); i++) {
        std::cout << tp_new.trajectory_points[i] << std::endl;
        EXPECT_TRUE(tp.trajectory_points[i].target_time < tp_new.trajectory_points[i].target_time);
    }

    EXPECT_EQ(7, tp.trajectory_points.size());

}

TEST(InLaneCruisingPluginTest, test_update_traj2)
{
    cav_msgs::TrajectoryPlan original_tp;

    cav_msgs::TrajectoryPlanPoint trajectory_point_1;
    cav_msgs::TrajectoryPlanPoint trajectory_point_2;
    cav_msgs::TrajectoryPlanPoint trajectory_point_3;
    cav_msgs::TrajectoryPlanPoint trajectory_point_4;
    cav_msgs::TrajectoryPlanPoint trajectory_point_5;
    cav_msgs::TrajectoryPlanPoint trajectory_point_6;
    cav_msgs::TrajectoryPlanPoint trajectory_point_7;

    trajectory_point_1.x = 1.0;
    trajectory_point_1.y = 1.0;
    trajectory_point_1.target_time = ros::Time(0);

    trajectory_point_2.x = 5.0;
    trajectory_point_2.y = 1.0;
    trajectory_point_2.target_time = ros::Time(1);

    trajectory_point_3.x = 10.0;
    trajectory_point_3.y = 1.0;
    trajectory_point_3.target_time = ros::Time(2);

    trajectory_point_4.x = 15.0;
    trajectory_point_4.y = 1.0;
    trajectory_point_4.target_time = ros::Time(3);

    trajectory_point_5.x = 20.0;
    trajectory_point_5.y = 1.0;
    trajectory_point_5.target_time = ros::Time(4);

    trajectory_point_6.x = 25.0;
    trajectory_point_6.y = 1.0;
    trajectory_point_6.target_time = ros::Time(5);

    trajectory_point_7.x = 30.0;
    trajectory_point_7.y = 1.0;
    trajectory_point_7.target_time = ros::Time(6);

    original_tp.trajectory_points = {trajectory_point_1, trajectory_point_2, trajectory_point_3, trajectory_point_4, trajectory_point_5, trajectory_point_6, trajectory_point_7};


    double initial_pos = 0.0;
    double goal_pos = 30.0;
    double current_speed_ = 10.0;
    double goal_velocity = 5.0;
    double initial_accel = 0.0;
    double goal_accel = -1;
    double initial_time = 0.0;
    double tp = 5.0;

    std::vector<double> values = quintic_coefficient_calculator::quintic_coefficient_calculator(initial_pos, 
                                                                                                goal_pos, 
                                                                                                current_speed_, 
                                                                                                goal_velocity, 
                                                                                                initial_accel,
                                                                                                goal_accel, 
                                                                                                initial_time, 
                                                                                                tp);

    std::vector<cav_msgs::TrajectoryPlanPoint> new_trajectory_points;

    new_trajectory_points.push_back(original_tp.trajectory_points[0]);

    object_avoidance::ObjectAvoidance obj;  

    std::vector<double> original_traj_downtracks = obj.get_relative_downtracks(original_tp);

    std::vector<double> new_speeds;
    std::vector<double> distance_travelled;


    for(size_t i = 1; i < original_tp.trajectory_points.size() ; i++ )
    {            
        double traj_target_time = i * tp / original_tp.trajectory_points.size();
        double dt_dist = obj.polynomial_calc(values, traj_target_time);
        distance_travelled.push_back(dt_dist);
        double dv = obj.polynomial_calc_d(values, traj_target_time);
        new_speeds.push_back(dv);
        cav_msgs::TrajectoryPlanPoint new_tpp;
        new_tpp.x = original_tp.trajectory_points[i].x;
        new_tpp.y = original_tp.trajectory_points[i].y;
        new_tpp.target_time = new_trajectory_points[0].target_time + ros::Duration(original_traj_downtracks[i]/dv);
        new_trajectory_points.push_back(new_tpp);
    }

    EXPECT_EQ(6, new_speeds.size());
    EXPECT_NEAR(8.9, new_speeds[0], 0.1);
    EXPECT_NEAR(6.9, new_speeds[1], 0.1);
    EXPECT_NEAR(5.1, new_speeds[2], 0.1);
    EXPECT_NEAR(4.2, new_speeds[3], 0.1);
    EXPECT_NEAR(4.3, new_speeds[4], 0.1);
    EXPECT_NEAR(4.9, new_speeds[5], 0.1);

    EXPECT_EQ(6, distance_travelled.size());
    EXPECT_TRUE(distance_travelled[5] < goal_pos );


}
