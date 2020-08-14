/*
 * Copyright (C) 2020 LEIDOS.
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

//using PublishObjectCallback = std::function<void(const cav_msgs::ExternalObjectList&)>;


namespace object
{


    TEST(MotionComputationWorker, Constructor)
    {   
        MotionComputationWorker([](const cav_msgs::ExternalObjectList& obj_pub){});

    }

    TEST(MotionComputationWorker, motionPredictionCallback)
    {    
        object::MotionComputationWorker mcw([&](const cav_msgs::ExternalObjectList& obj_pub){});


        cav_msgs::ExternalObject msg;

        /*Create test message*/
        msg.presence_vector = 16;
        msg.object_type = 3;

        /*Test ExternalObject Presence Vector Values*/
        ASSERT_TRUE(msg.presence_vector > 0);
    
        //uint16_t pv_Values[10] ={1, 2, 4, 8, 16, 32, 64, 128, 256, 512};
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

        cav_msgs::ExternalObjectListPtr obj_list(new cav_msgs::ExternalObjectList(obj));

        mcw.predictionLogic(obj_list);
        bool isFilled = false;
        for(auto item : obj_list->objects)
        {
            if(item.predictions.size() > 0);
                isFilled = true;
        }
        ROS_INFO_STREAM("Test Predictions");
        //ASSERT_TRUE(obj_list.get()->objects[0].predictions.size() > 0);    //Create Assertion Statement to test whether object.prediction is empty
        ASSERT_EQ(isFilled, true);
        mcw.motionPredictionCallback(obj_list);

    }






}
