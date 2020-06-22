
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
#include <ros/ros.h>
#include <string>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include "platoon_strategic.hpp"

namespace platoon_strategic
{
    PlatoonStrategicPlugin::PlatoonStrategicPlugin(){}
                                    

    void PlatoonStrategicPlugin::initialize()
    {
        nh_.reset(new ros::CARMANodeHandle());
        pnh_.reset(new ros::CARMANodeHandle("~"));
        
        trajectory_srv_ = nh_->advertiseService("plugins/PlatoonStrategicPlugin/plan_trajectory", &PlatoonStrategicPlugin::plan_trajectory_cb, this);
                
        platoon_strategic_plugin_discovery_pub_ = nh_->advertise<cav_msgs::Plugin>("plugin_discovery", 1);
        plugin_discovery_msg_.name = "PlatoonStrategicPlugin";
        plugin_discovery_msg_.versionId = "v1.0";
        plugin_discovery_msg_.available = true;
        plugin_discovery_msg_.activated = false;
        plugin_discovery_msg_.type = cav_msgs::Plugin::TACTICAL;
        plugin_discovery_msg_.capability = "tactical_plan/plan_trajectory";
        
        pose_sub_ = nh_->subscribe("current_pose", 1, &PlatoonStrategicPlugin::pose_cb, this);
        pnh_->param<double>("trajectory_time_length", trajectory_time_length_, 6.0);
        pnh_->param<std::string>("control_plugin_name", control_plugin_name_, "NULL");

        ros::CARMANodeHandle::setSpinCallback([this]() -> bool {
            platoon_strategic_plugin_discovery_pub_.publish(plugin_discovery_msg_);
            return true;
        });


    }

    void PlatoonStrategicPlugin::states(){
        switch (psm_->current_platoon_state)
        {
        case PlatoonState::STANDBY:
            run_standby();
            break;
        case PlatoonState::LEADERWAITING:
            run_leader_waiting();
            break;

        case PlatoonState::LEADER:
            run_leader();
            break;
        
        case PlatoonState::CANDIDATEFOLLOWER:
            run_candidate_follower();
            break;
        
        case PlatoonState::FOLLOWER:
            run_follower();
            break;
        
        default:
            throw std::invalid_argument("Invalid State");
            break;
        }


    }

    void PlatoonStrategicPlugin::run()
    {
    	initialize();
        ros::CARMANodeHandle::setSpinRate(10.0);
        ros::CARMANodeHandle::spin();

    }

    void PlatoonStrategicPlugin::pose_cb(const geometry_msgs::PoseStampedConstPtr& msg)
    {
        pose_msg_ = msg;
    }

    bool PlatoonStrategicPlugin::plan_trajectory_cb(cav_srvs::PlanTrajectoryRequest &req, cav_srvs::PlanTrajectoryResponse &resp){
        

        return true;
    }


    void PlatoonStrategicPlugin::run_standby(){

    }

    void PlatoonStrategicPlugin::run_leader_waiting(){
        LeaderState ls;
        // ls.onMobilityOperationMessage(msg);
    }

    void PlatoonStrategicPlugin::run_leader(){
        
    }

    void PlatoonStrategicPlugin::run_follower(){
        
    }

    void PlatoonStrategicPlugin::run_candidate_follower(){
        
    }
}
