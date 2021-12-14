/*
 * Copyright (C) 2020-2021 LEIDOS.
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


#include <gmock/gmock.h>
#include <cav_msgs/ExternalObject.h>
#include <cav_msgs/ExternalObjectList.h>
#include "motion_computation_worker.h"  
#include <functional>

#include <memory>
#include <chrono>
#include <ctime>
#include <atomic>

using ::testing::_;
using ::testing::A;
using ::testing::DoAll;
using ::testing::InSequence;
using ::testing::Return;
using ::testing::ReturnArg;
#include <ros/ros.h>
//using PublishObjectCallback = std::function<void(const cav_msgs::ExternalObjectList&)>;


namespace object
{


    TEST(MotionComputationWorker, Constructor)
    {   
        MotionComputationWorker([](const cav_msgs::ExternalObjectList& obj_pub){});

    }

    TEST(MotionComputationWorker, motionPredictionCallback)
    {    
        bool published_data = false;
        object::MotionComputationWorker mcw_sensor_only([&](const cav_msgs::ExternalObjectList& obj_pub){
            published_data = true;
            ASSERT_EQ(obj_pub.objects.size(), 1);

            bool isFilled = false;
            for(auto item : obj_pub.objects)
            {
                if(item.predictions.size() > 0);
                    isFilled = true;
            }
            ROS_INFO_STREAM("Test Predictions");
            ASSERT_EQ(isFilled, true);

            });     
        object::MotionComputationWorker mcw_mobility_only([&](const cav_msgs::ExternalObjectList& obj_pub){
            published_data = true;
            ASSERT_EQ(obj_pub.objects.size(), 0);
        });
        object::MotionComputationWorker mcw_mixed_operation([&](const cav_msgs::ExternalObjectList& obj_pub){
            published_data = true;
            ASSERT_EQ(obj_pub.objects.size(), 2);
        });
        mcw_sensor_only.setExternalObjectPredictionMode(object::SENSORS_ONLY);
        mcw_mobility_only.setExternalObjectPredictionMode(object::MOBILITY_PATH_ONLY);
        mcw_mixed_operation.setExternalObjectPredictionMode(object::PATH_AND_SENSORS);

        // 1 to 1 transform
        std::string base_proj = lanelet::projection::LocalFrameProjector::ECEF_PROJ_STR;
        std_msgs::String proj_msg;
        proj_msg.data = base_proj;
        std_msgs::StringConstPtr msg_ptr(new std_msgs::String(proj_msg));
        mcw_sensor_only.georeferenceCallback(msg_ptr);  // Set projection
        mcw_mobility_only.georeferenceCallback(msg_ptr);  // Set projection
        mcw_mixed_operation.georeferenceCallback(msg_ptr);  // Set projection

        cav_msgs::ExternalObject msg;

        /*Create test message*/
        msg.presence_vector = 16;
        msg.object_type = 3;

        /*Test ExternalObject Presence Vector Values*/
        ASSERT_TRUE(msg.presence_vector > 0);

        
        bool pvValid = false;
        for(auto i= 0; i<10; i++) //Test whether presence vector values in ExternalObject are valid
        {
            if (msg.presence_vector == pow(2,i))//presence vector is valid if it matches binary value between 1-512
                pvValid = true;
        }
        ASSERT_EQ(pvValid, true);

        /*Test ExternalObject Object Type Values*/
        bool otValid = false;
        for(int i =0; i<=4; i++)
        {
            if(msg.object_type == i)
                otValid = true;
        }
        ASSERT_EQ(otValid, true);

        /*Test ExternalObjectList*/
        cav_msgs::ExternalObjectList obj;
        

        ROS_INFO_STREAM("Before push back");
        obj.objects.push_back(msg);
        ROS_INFO_STREAM("Push back successful");
        ASSERT_TRUE(obj.objects.size() > 0);

        // add mobilitypath data
        cav_msgs::MobilityPath input_path;

        // INPUT PATH
        input_path.m_header.sender_bsm_id = "FFFFFFFF";
        input_path.m_header.timestamp = 1000;
        input_path.trajectory.location.ecef_x = 0; //local map 0,0,0
        input_path.trajectory.location.ecef_y = 0; 
        input_path.trajectory.location.ecef_z = 0; 
        
        cav_msgs::LocationOffsetECEF location;
        location.offset_x = 0;
        location.offset_y = 0;
        location.offset_z = 0;

        input_path.trajectory.offsets.push_back(location);
        mcw_sensor_only.mobilityPathCallback(input_path); // added a mobilitypath which won't be processed

        cav_msgs::ExternalObjectListPtr obj_list(new cav_msgs::ExternalObjectList(obj));
        
        // Check sensor only mode
        mcw_sensor_only.predictionLogic(obj_list);
        //assert published_data is true
        ASSERT_EQ(published_data, true);

        // check mixed operation mode
        published_data = false;
        mcw_mixed_operation.mobilityPathCallback(input_path);
        mcw_mixed_operation.predictionLogic(obj_list);
        //assert published_data is true
        ASSERT_EQ(published_data, true);
        
    }

    TEST(MotionComputationWorker, composePredictedState)
    {    
        object::MotionComputationWorker mcw([&](const cav_msgs::ExternalObjectList& obj_pub){});

        // 1 to 1 transform
        std::string base_proj = lanelet::projection::LocalFrameProjector::ECEF_PROJ_STR;
        std_msgs::String proj_msg;
        proj_msg.data = base_proj;
        std_msgs::StringConstPtr msg_ptr(new std_msgs::String(proj_msg));
        mcw.georeferenceCallback(msg_ptr);  // Set projection

        ros::Time time_stamp = ros::Time(5.0);
        tf2::Vector3 curr = {5, 0, 0};
        tf2::Vector3 prev = {4, 0, 0};

        auto res =  mcw.composePredictedState(curr, prev,time_stamp, 0.0);
        auto test_result = std::get<0>(res);

        ASSERT_NEAR(test_result.predicted_position.position.x, 4.0, 0.0001 );
        ASSERT_NEAR(test_result.predicted_position.orientation.w, 1.0, 0.0001 );
        ASSERT_NEAR(test_result.predicted_position.orientation.x, 0.0, 0.0001 );
        ASSERT_NEAR(test_result.predicted_position.orientation.y, 0.0, 0.0001 );
        ASSERT_NEAR(test_result.predicted_position.orientation.z, 0.0, 0.0001 );
        ASSERT_NEAR(test_result.predicted_velocity.linear.x, 10.0, 0.0001 );
        ASSERT_EQ(test_result.header.stamp, time_stamp);

        curr = {5, 5, 0};
        prev = {0, 0, 0};

        res =  mcw.composePredictedState(curr, prev,time_stamp, std::get<1>(res));
        test_result = std::get<0>(res);

        ASSERT_NEAR(test_result.predicted_position.position.x, 0.0, 0.0001 );
        ASSERT_NEAR(test_result.predicted_position.position.y, 0.0, 0.0001 );
        ASSERT_NEAR(test_result.predicted_position.orientation.w, 0.9238, 0.0001 );
        ASSERT_NEAR(test_result.predicted_velocity.linear.x, 5.0*sqrt(2)/0.1 , 0.0001 );
        ASSERT_EQ(test_result.header.stamp, time_stamp);

    }

    TEST(MotionComputationWorker, mobilityPathToExternalObject)
    {    
        object::MotionComputationWorker mcw([&](const cav_msgs::ExternalObjectList& obj_pub){});

        // 1 to 1 transform
        std::string base_proj = lanelet::projection::LocalFrameProjector::ECEF_PROJ_STR;
        std_msgs::String proj_msg;
        proj_msg.data = base_proj;
        std_msgs::StringConstPtr msg_ptr(new std_msgs::String(proj_msg));
        mcw.georeferenceCallback(msg_ptr);  // Set projection

        // Test no georef
        cav_msgs::MobilityPath input;
        cav_msgs::ExternalObject output, expected;
        output = mcw.mobilityPathToExternalObject(input);
        ASSERT_EQ(output.header.stamp, expected.header.stamp); //empty object returned

        // INPUT
        input.m_header.sender_bsm_id = "FFFFFFFF";
        input.m_header.timestamp = 1000;
        input.trajectory.location.ecef_x = 0; //local map 0,0,0
        input.trajectory.location.ecef_y = 0; 
        input.trajectory.location.ecef_z = 0; 
        
        cav_msgs::LocationOffsetECEF location;
        location.offset_x = 0;
        location.offset_y = 0;
        location.offset_z = 0;

        input.trajectory.offsets.push_back(location); // First point has no movement
        
        // Test static/dynamic
        output = mcw.mobilityPathToExternalObject(input);
        ASSERT_FALSE(output.dynamic_obj);
        
        location.offset_x = 500.00; 
        location.offset_y = 0;
        location.offset_z = 0;
        input.trajectory.offsets.push_back(location); // Second point moves 5m forward

        // Test 0th, 1st point predicted state
        output = mcw.mobilityPathToExternalObject(input);
        ASSERT_NEAR(output.pose.pose.orientation.w, 1.0, 0.0001);
        ASSERT_NEAR(output.pose.pose.position.x, 0.0, 0.005);
        ASSERT_NEAR(output.velocity.twist.linear.x, 0.0, 0.1);

        ASSERT_NEAR(output.predictions[0].predicted_position.orientation.w, 1.0, 0.0001);
        ASSERT_NEAR(output.predictions[0].predicted_position.position.x, 0.0, 0.03);
        ASSERT_NEAR(output.predictions[0].predicted_velocity.linear.x, 50.0, 0.1); // TODO VELOCITY ISSUE it really doesn't make sense that we can not move but suddenly have 50m/s velocity. Discuss with reviewer

        ASSERT_NEAR(output.predictions[1].predicted_position.orientation.w, 1.0, 0.0001);
        ASSERT_NEAR(output.predictions[1].predicted_position.position.x, 5.0, 0.03);
        ASSERT_NEAR(output.predictions[1].predicted_velocity.linear.x, 50.0, 0.1);

        ASSERT_EQ(output.header.stamp, ros::Time(1.0));
        ROS_WARN_STREAM("time: " << output.predictions[0].header.stamp.toSec());
        ASSERT_EQ(output.predictions[0].header.stamp, ros::Time(1.1));

    }

    TEST(MotionComputationWorker, synchronizeAndAppend)
    {    
        object::MotionComputationWorker mcw_mixed_operation([&](const cav_msgs::ExternalObjectList& obj_pub){});
        mcw_mixed_operation.setExternalObjectPredictionMode(object::PATH_AND_SENSORS);
        mcw_mixed_operation.setMobilityPathPredictionTimeStep(0.2);

        // 1 to 1 transform
        std::string base_proj = lanelet::projection::LocalFrameProjector::ECEF_PROJ_STR;
        std_msgs::String proj_msg;
        proj_msg.data = base_proj;
        std_msgs::StringConstPtr msg_ptr(new std_msgs::String(proj_msg));
        mcw_mixed_operation.georeferenceCallback(msg_ptr);  // Set projection

        // Test no georef
        cav_msgs::ExternalObject sensor_obj, mobility_path_obj;
        sensor_obj.header.stamp = ros::Time(1.6);
        mobility_path_obj.header.stamp = ros::Time(1.1);
        mobility_path_obj.pose.pose.orientation.w = 1;
        mobility_path_obj.pose.pose.orientation.x = 0;
        mobility_path_obj.pose.pose.orientation.y = 0;
        mobility_path_obj.pose.pose.orientation.z = 0;
        mobility_path_obj.pose.pose.position.x = 200;
        mobility_path_obj.pose.pose.position.y = 0;
        mobility_path_obj.pose.pose.position.z = 0;
        mobility_path_obj.velocity.twist.linear.x = 2000;

        cav_msgs::PredictedState next_state;                // 0
        next_state.header.stamp = ros::Time(1.3);
        next_state.predicted_position.orientation.w = 1;
        next_state.predicted_position.orientation.x = 0;
        next_state.predicted_position.orientation.y = 0;
        next_state.predicted_position.orientation.z = 0;
        next_state.predicted_position.position.x = 200;
        next_state.predicted_position.position.y = 0;
        next_state.predicted_position.position.z = 0;
        next_state.predicted_velocity.linear.x = 2000;
        mobility_path_obj.predictions.push_back(next_state); //1
        next_state.header.stamp = ros::Time(1.5);
        next_state.predicted_position.position.x = 400;
        mobility_path_obj.predictions.push_back(next_state); //2
        next_state.header.stamp = ros::Time(1.7);
        next_state.predicted_position.position.x = 600;
        mobility_path_obj.predictions.push_back(next_state); //3
        next_state.header.stamp = ros::Time(1.9);
        next_state.predicted_position.position.x = 800;
        mobility_path_obj.predictions.push_back(next_state); //4
        next_state.header.stamp = ros::Time(2.1);
        next_state.predicted_position.position.x = 1000;
        mobility_path_obj.predictions.push_back(next_state); //5

        cav_msgs::ExternalObjectList sensor_list, mobility_path_list;
        sensor_list.objects.push_back(sensor_obj);
        mobility_path_list.objects.push_back(mobility_path_obj);
        auto result = mcw_mixed_operation.synchronizeAndAppend(sensor_list, mobility_path_list);
        
        ASSERT_EQ(result.objects.size(), 2);
        ASSERT_EQ(result.objects[1].predictions.size(), 2); // we dropped 2 points
        ASSERT_EQ(result.objects[1].predictions[0].header.stamp, ros::Time(1.8));
        ASSERT_EQ(result.objects[1].predictions[0].predicted_position.position.x, 700);
        ASSERT_EQ(result.objects[1].predictions[1].header.stamp, ros::Time(2.0));
        ASSERT_EQ(result.objects[1].predictions[1].predicted_position.position.x, 900);

    }


} // namespace object
