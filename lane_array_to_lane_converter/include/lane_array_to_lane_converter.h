#pragma once

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

#include <ros/ros.h>
#include <autoware_msgs/LaneArray.h>
#include <autoware_msgs/Lane.h>
#include <carma_utils/CARMAUtils.h>
#include <visualization_msgs/MarkerArray.h>

namespace lane_array_to_lane_converter {

    /**
     * LaneArrayToLaneConverter converts and publishes autoware_msgs/LaneArray.msg to autoware_msgs/Lane.msg by picking the first lane
     * in the array as it simply assumes the whole array is one lane.
     * 
    */ 
    class LaneArrayToLaneConverter
    {

    public:

        /**
         * \brief Default constructor
         */
        LaneArrayToLaneConverter();

        /**
         * \brief General starting point to run this node
         */
        void run();

    private:

        // public and private node handles
        std::shared_ptr<ros::CARMANodeHandle> nh_, pnh_;

        // publisher
        ros::Publisher lane_pub_;
        
        // subscriber
        ros::Subscriber lane_array_sub_;
        
        // initialize this node before running
        void initialize();

        // callbacks
        void callbackFromLaneArray(const autoware_msgs::LaneArrayConstPtr& msg);

        // variables
        autoware_msgs::Lane current_lane_;  // current lane we are driving
    };

}


