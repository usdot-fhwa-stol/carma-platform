
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
#include "platoon_strategic.hpp"


namespace platoon_strategic
{
    PlatoonStrategicPlugin::PlatoonStrategicPlugin(){}

    PlatoonStrategicPlugin::PlatoonStrategicPlugin(carma_wm::WorldModelConstPtr wm, PlatoonPluginConfig config,
                                PublishPluginDiscoveryCB plugin_discovery_publisher, MobilityResponseCB mobility_response_publisher,
                                MobilityRequestCB mobility_request_publisher, MobilityOperationCB mobility_operation_publisher,
                                PlatooningInfoCB platooning_info_publisher)
    : wm_(wm), config_(config), plugin_discovery_publisher_(plugin_discovery_publisher), 
      mobility_response_publisher_(mobility_response_publisher), mobility_request_publisher_(mobility_request_publisher),
      mobility_operation_publisher_(mobility_operation_publisher), platooning_info_publisher_(platooning_info_publisher)
    {
        plugin_discovery_msg_.name = "YieldPlugin";
        plugin_discovery_msg_.versionId = "v1.0";
        plugin_discovery_msg_.available = true;
        plugin_discovery_msg_.activated = false;
        plugin_discovery_msg_.type = cav_msgs::Plugin::STRATEGIC;
        plugin_discovery_msg_.capability = "strategic_plan/plan_maneuvers";  
    }
                                    

    // void PlatoonStrategicPlugin::initialize()
    // {
    //     nh_.reset(new ros::CARMANodeHandle());
    //     pnh_.reset(new ros::CARMANodeHandle("~"));

        
        
    //     maneuver_srv_ = nh_->advertiseService("strategic_plan/plan_maneuvers", &PlatoonStrategicPlugin::plan_maneuver_cb, this);

    //     mob_op_pub_ = nh_->advertise<cav_msgs::MobilityOperation>("mobility_operation_message", 5);
    //     mob_req_pub_ = nh_->advertise<cav_msgs::MobilityRequest>("mobility_request_message", 5);
                
    //     platoon_strategic_plugin_discovery_pub_ = nh_->advertise<cav_msgs::Plugin>("plugin_discovery", 1);
    //     plugin_discovery_msg_.name = "PlatooningStrategicPlugin";
    //     plugin_discovery_msg_.versionId = "v1.0";
    //     plugin_discovery_msg_.available = true;
    //     plugin_discovery_msg_.activated = false;
    //     plugin_discovery_msg_.type = cav_msgs::Plugin::STRATEGIC;
    //     plugin_discovery_msg_.capability = "strategic_plan/plan_maneuvers";
        
    //     pose_sub_ = nh_->subscribe("current_pose", 1, &PlatoonStrategicPlugin::pose_cb, this);

    //     mob_req_sub_ = nh_->subscribe("incoming_mobility_request_message", 1, &PlatoonStrategicPlugin::mob_req_cb, this);
    //     mob_resp_sub_ = nh_->subscribe("incoming_mobility_response_message", 1, &PlatoonStrategicPlugin::mob_resp_cb, this);
    //     mob_op_sub_ = nh_->subscribe("incoming_mobility_operation_message", 1, &PlatoonStrategicPlugin::mob_op_cb, this);


    //     ros::CARMANodeHandle::setSpinCallback([this]() -> bool {
    //         platoon_strategic_plugin_discovery_pub_.publish(plugin_discovery_msg_);
    //         return true;
    //     });


    // }

    bool PlatoonStrategicPlugin::onSpin() 
    {
        plugin_discovery_publisher_(plugin_discovery_msg_);
        return true;
    }

    void PlatoonStrategicPlugin::run_states(){
        
        switch (psm_.current_platoon_state)
        {
        case PlatoonState::STANDBY:
            break;
        case PlatoonState::LEADERWAITING:
            waitingStartTime = ros::Time::now().toSec()*1000;
            run_leader_waiting();
            break;

        case PlatoonState::LEADER:
            run_leader();
            break;
        
        case PlatoonState::CANDIDATEFOLLOWER:
            candidatestateStartTime = ros::Time::now().toSec()*1000;
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


    // void PlatoonStrategicPlugin::run()
    // {
    // 	// initialize();
    //     psm_.onMobilityRequestMessage(mobility_req_msg_);
    //     psm_.onMobilityOperationMessage(mobility_op_msg_);
    //     psm_.onMobilityResponseMessage(mobility_resp_msg_);
    //     run_states();
    //     ros::CARMANodeHandle::setSpinRate(10.0);
    //     ros::CARMANodeHandle::spin();

    // }

    int PlatoonStrategicPlugin::findLaneletIndexFromPath(int target_id, lanelet::routing::LaneletPath& path)
    {
        for(size_t i = 0; i < path.size(); ++i)
        {
            if(path[i].id() == target_id)
            {
                return i;
            }
        }
        return -1;
    }

    void PlatoonStrategicPlugin::bsm_cb(const cav_msgs::BSMConstPtr& msg)
    {
        cav_msgs::BSMCoreData bsm_core_ = msg->core_data;
        host_bsm_id_ = bsmIDtoString(bsm_core_);
    }

    void PlatoonStrategicPlugin::pose_cb(const geometry_msgs::PoseStampedConstPtr& msg)
    {
        pose_msg_ = geometry_msgs::PoseStamped(*msg.get());
    }

    void PlatoonStrategicPlugin::twist_cb(const geometry_msgs::TwistStampedConstPtr& msg)
    {
        current_speed_ = msg->twist.linear.x;
    }

    void PlatoonStrategicPlugin::updateCurrentStatus(cav_msgs::Maneuver maneuver, double& speed, double& current_progress, int& lane_id){
        if(maneuver.type == cav_msgs::Maneuver::LANE_FOLLOWING){
            speed =  maneuver.lane_following_maneuver.end_speed;
            current_progress =  maneuver.lane_following_maneuver.end_dist;
            lane_id =  stoi(maneuver.lane_following_maneuver.lane_id);
        }
    }

    

    bool PlatoonStrategicPlugin::plan_maneuver_cb(cav_srvs::PlanManeuversRequest &req, cav_srvs::PlanManeuversResponse &resp)
    {
        lanelet::BasicPoint2d current_loc(pose_msg_.pose.position.x, pose_msg_.pose.position.y);
        auto current_lanelets = lanelet::geometry::findNearest(wm_->getMap()->laneletLayer, current_loc, 10);       
        if(current_lanelets.size() == 0)
        {
            ROS_WARN_STREAM("Cannot find any lanelet in map!");
            return true;
        }
        
        auto shortest_path = wm_->getRoute()->shortestPath();

        lanelet::ConstLanelet current_lanelet;
        int last_lanelet_index = -1;
        for (auto llt : current_lanelets)
        {
            if (boost::geometry::within(current_loc, llt.second.polygon2d()))
            {
                int potential_index = findLaneletIndexFromPath(llt.second.id(), shortest_path);
                if (potential_index != -1)
                {
                    last_lanelet_index = potential_index;
                    current_lanelet = shortest_path[last_lanelet_index];
                    break;
                }
            }
        }
        if(last_lanelet_index == -1)
        {
            ROS_ERROR_STREAM("Current position is not on the shortest path! Returning an empty maneuver");
            return true;
        }
        double current_progress = wm_->routeTrackPos(current_loc).downtrack;
        double speed_progress = current_speed_;
        ros::Time time_progress = ros::Time::now();
        double target_speed = findSpeedLimit(current_lanelet);   //get Speed Limit TOTO update

        double total_maneuver_length = current_progress + config_.mvr_duration * target_speed;
        double route_length =  wm_->getRouteEndTrackPos().downtrack; 
        total_maneuver_length = std::min(total_maneuver_length, route_length);

        //Update current status based on prior plan
        if(req.prior_plan.maneuvers.size()!=0){
            time_progress = req.prior_plan.planning_completion_time;
            int end_lanelet =0;
            updateCurrentStatus(req.prior_plan.maneuvers.back(),speed_progress,current_progress,end_lanelet);
            last_lanelet_index = findLaneletIndexFromPath(end_lanelet,shortest_path);
        }
        bool approaching_route_end = false;
        double time_req_to_stop,stopping_dist;

        ROS_DEBUG_STREAM("Starting Loop");
        ROS_DEBUG_STREAM("total_maneuver_length: " << total_maneuver_length << " route_length: " << route_length);
        while(current_progress < total_maneuver_length)
        {
            ROS_DEBUG_STREAM("Lanlet: " << shortest_path[last_lanelet_index].id());
            ROS_DEBUG_STREAM("current_progress: "<< current_progress);
            ROS_DEBUG_STREAM("speed_progress: " << speed_progress);
            ROS_DEBUG_STREAM("target_speed: " << target_speed);
            ROS_DEBUG_STREAM("time_progress: " << time_progress.toSec());
            auto p = shortest_path[last_lanelet_index].centerline2d().back();
            double end_dist = wm_->routeTrackPos(shortest_path[last_lanelet_index].centerline2d().back()).downtrack;
            end_dist = std::min(end_dist, total_maneuver_length);
            ROS_DEBUG_STREAM("end_dist: " << end_dist);
            double dist_diff = end_dist - current_progress;
            ROS_DEBUG_STREAM("dist_diff: " << dist_diff);
            if(end_dist < current_progress){
                break;
            }

            resp.new_plan.maneuvers.push_back(composeManeuverMessage(current_progress, end_dist,  
                                    speed_progress, target_speed,shortest_path[last_lanelet_index].id(), time_progress));
            
            current_progress += dist_diff;
            time_progress = resp.new_plan.maneuvers.back().lane_following_maneuver.end_time;
            speed_progress = target_speed;
            if(current_progress >= total_maneuver_length || last_lanelet_index == shortest_path.size() - 1)
            {
                break;
            }

            ++last_lanelet_index;
        }

        if(resp.new_plan.maneuvers.size() == 0)
        {
            ROS_WARN_STREAM("Cannot plan maneuver because no route is found");
        }        
        return true;
    }

    double PlatoonStrategicPlugin::findSpeedLimit(const lanelet::ConstLanelet& llt)
    {
        lanelet::Optional<carma_wm::TrafficRulesConstPtr> traffic_rules = wm_->getTrafficRules();
        double target_speed = 0.0, traffic_speed =0.0, param_speed =0.0;
        double hardcoded_max=lanelet::Velocity(hardcoded_params::control_limits::MAX_LONGITUDINAL_VELOCITY_MPS * lanelet::units::MPS()).value();

        if (traffic_rules)
        {
            traffic_speed=(*traffic_rules)->speedLimit(llt).speedLimit.value();
            
        }
        else{
            ROS_WARN(" Valid traffic rules object could not be built.");
        }

        if(config_.config_limit > 0.0 && config_.config_limit < hardcoded_max)
        {
            param_speed = config_.config_limit;
            ROS_DEBUG("Using Configurable value");
        }
        else 
        {
            param_speed = hardcoded_max;
            ROS_DEBUG(" Using Hardcoded maximum");
        }
        //If either value is 0, use the other valid limit
        if(traffic_speed <= config_.epislon || param_speed <= config_.epislon){
            target_speed = std::max(traffic_speed, param_speed);
        }
        else{
            target_speed = std::min(traffic_speed,param_speed);
        }
        
        return target_speed;
    }

    cav_msgs::Maneuver PlatoonStrategicPlugin::composeManeuverMessage(double current_dist, double end_dist, double current_speed, double target_speed, int lane_id, ros::Time& current_time)
    {
        cav_msgs::Maneuver maneuver_msg;
        maneuver_msg.type = cav_msgs::Maneuver::LANE_FOLLOWING;
        maneuver_msg.lane_following_maneuver.parameters.neogition_type = cav_msgs::ManeuverParameters::PLATOONING;
        maneuver_msg.lane_following_maneuver.parameters.presence_vector = cav_msgs::ManeuverParameters::HAS_TACTICAL_PLUGIN;
        maneuver_msg.lane_following_maneuver.parameters.planning_tactical_plugin = "PlatooningTacticalPlugin";
        maneuver_msg.lane_following_maneuver.parameters.planning_strategic_plugin = "PlatooningStrategicPlugin";
        maneuver_msg.lane_following_maneuver.start_dist = current_dist;
        maneuver_msg.lane_following_maneuver.start_speed = current_speed;
        maneuver_msg.lane_following_maneuver.start_time = current_time;
        maneuver_msg.lane_following_maneuver.end_dist = end_dist;
        maneuver_msg.lane_following_maneuver.end_speed = target_speed;
        
        // because it is a rough plan, assume vehicle can always reach to the target speed in a lanelet
        double cur_plus_target = current_speed + target_speed;
        if (cur_plus_target < 0.00001) {
            maneuver_msg.lane_following_maneuver.end_time = current_time + ros::Duration(config_.mvr_duration);
        } else {
            maneuver_msg.lane_following_maneuver.end_time = current_time + ros::Duration((end_dist - current_dist) / (0.5 * cur_plus_target));
        }
        maneuver_msg.lane_following_maneuver.lane_id = std::to_string(lane_id);
        current_time = maneuver_msg.lane_following_maneuver.end_time;
        ROS_INFO_STREAM("Creating lane follow start dist:"<<current_dist<<" end dist:"<<end_dist);
        ROS_INFO_STREAM("Duration:"<<maneuver_msg.lane_following_maneuver.end_time.toSec() - maneuver_msg.lane_following_maneuver.start_time.toSec());
        return maneuver_msg;
    }



    void PlatoonStrategicPlugin::run_leader_waiting(){
        long tsStart = ros::Time::now().toSec()*1000;
            // Task 1
                if(tsStart - waitingStartTime > waitingStateTimeout * 1000) {
                    //TODO if the current state timeouts, we need to have a kind of ABORT message to inform the applicant
                    ROS_DEBUG("LeaderWaitingState is timeout, changing back to PlatoonLeaderState.");
                    // plugin.setState(new LeaderState(plugin, log, pluginServiceLocator));
                    psm_.current_platoon_state = PlatoonState::LEADER;
                }
                // Task 2
                cav_msgs::MobilityOperation status;
                composeMobilityOperationLeaderWaiting(status);
                mob_op_pub_.publish(status);
                long tsEnd = ros::Time::now().toSec()*1000;
                int sleepDuration = std::max(int(statusMessageInterval_ - (tsEnd - tsStart)), 0);
                // ros::Duration(sleepDuration/1000).sleep();
    }

    void PlatoonStrategicPlugin::run_leader(){

        long tsStart = ros::Time::now().toSec()*1000;
            // Task 1
            bool isTimeForHeartBeat = tsStart - lastHeartBeatTime >= infoMessageInterval_;
            if(isTimeForHeartBeat) {
                    cav_msgs::MobilityOperation infoOperation;
                    composeMobilityOperationLeader(infoOperation, "INFO");
                    mob_op_pub_.publish(infoOperation);
                    lastHeartBeatTime = ros::Time::now().toSec()*1000.0;
                    ROS_DEBUG("Published heart beat platoon INFO mobility operatrion message");
                }
            // Task 2
            if (isTimeForHeartBeat) {
                    // updateLightBar();
            }
            // Task 3
            {
                // std::lock_guard<std::mutex> lock(plan_mutex_);
                if(psm_.current_plan.valid) {
                    bool isCurrentPlanTimeout = ((ros::Time::now().toSec()*1000 - psm_.current_plan.planStartTime) > NEGOTIATION_TIMEOUT);
                    if(isCurrentPlanTimeout) {
                        ROS_DEBUG("Give up current on waiting plan with planId: " , psm_.current_plan.planId);
                        psm_.current_plan.valid = false;
                    }    
                }
            }

            // Task 4
            bool hasFollower = (psm_.pm_.getTotalPlatooningSize() > 1);
            if(hasFollower) {
                cav_msgs::MobilityOperation statusOperation;
                composeMobilityOperationLeader(statusOperation, "STATUS");
                mob_op_pub_.publish(statusOperation);
                ROS_DEBUG("Published platoon STATUS operation message");
            }
            long tsEnd =  ros::Time::now().toSec()*1000;
            long sleepDuration = std::max(int(statusMessageInterval_ - (tsEnd - tsStart)), 0);
            // is sleep needed?
            // ros::Duration(sleepDuration/1000).sleep();
        
    }

    void PlatoonStrategicPlugin::run_follower(){
        // This is a interrupted-safe loop.
        // This loop has four tasks:
        // 1. Check the state start time, if it exceeds a limit it will give up current plan and change back to leader state
        // 2. Abort current request if we wait for long enough time for response from leader and change back to leader state
        // 3. Check the current distance with the target platoon rear and send out CANDIDATE-JOIN request when we get close
        // 4. Publish operation status every 100 milliseconds if we still have followers
        long tsStart = ros::Time::now().toSec()*1000;
            // Job 1
            cav_msgs::MobilityOperation status;
            composeMobilityOperationFollower(status);
            mob_op_pub_.publish(status);
            // Job 2
            // Get the number of vehicles in this platoon who is in front of us
            int vehicleInFront = psm_.pm_.getNumberOfVehicleInFront();
                if(vehicleInFront == 0) {
                    noLeaderUpdatesCounter++;
                    if(noLeaderUpdatesCounter >= LEADER_TIMEOUT_COUNTER_LIMIT) {
                        ROS_DEBUG("noLeaderUpdatesCounter = " , noLeaderUpdatesCounter , " and change to leader state");
                        psm_.pm_.changeFromFollowerToLeader();
                        psm_.current_platoon_state = PlatoonState::LEADER;
                        
                    }
                } else {
                    // reset counter to zero when we get updates again
                    noLeaderUpdatesCounter = 0;
                }
                long tsEnd = ros::Time::now().toSec()*1000.0;
                long sleepDuration = std::max(int(statusMessageInterval_ - (tsEnd - tsStart)), 0);
                ros::Duration(sleepDuration/1000).sleep();
        
    }

    void PlatoonStrategicPlugin::run_candidate_follower(){
        long tsStart = ros::Time::now().toSec()*1000.0; 
        // Task 1
        bool isCurrentStateTimeout = (tsStart - candidatestateStartTime) > waitingStateTimeout * 1000;
        if(isCurrentStateTimeout) {
            ROS_DEBUG("The current candidate follower state is timeout. Change back to leader state.");
            psm_.current_platoon_state = PlatoonState::LEADER;
        }
        // Task 2

        if(psm_.current_plan.valid) {
            {
                std::lock_guard<std::mutex> lock(plan_mutex_);
                if(psm_.current_plan.valid) {
                    bool isPlanTimeout = (tsStart - psm_.current_plan.planStartTime) > NEGOTIATION_TIMEOUT;
                    if(isPlanTimeout) {
                        psm_.current_plan.valid = false;
                        ROS_DEBUG("The current plan did not receive any response. Abort and change to leader state.");
                        psm_.current_platoon_state = PlatoonState::LEADER;
                    }    
                }
            }
        }

        // Task 3
                double desiredJoinGap2 = desiredJoinTimeGap;
                double maxJoinGap = std::max(desiredJoinGap, desiredJoinGap2);
                double currentGap = psm_.pm_.getDistanceToFrontVehicle();
                ROS_DEBUG("Based on desired join time gap, the desired join distance gap is " , desiredJoinGap2 , " ms");
                ROS_DEBUG("Since we have max allowed gap as " , desiredJoinGap , " m then max join gap became " , maxJoinGap , " m");
                ROS_DEBUG("The current gap from radar is " , currentGap , " m");
                if(currentGap <= maxJoinGap && psm_.current_plan.valid == false) {
                    cav_msgs::MobilityRequest request;
                    std::string planId = boost::uuids::to_string(boost::uuids::random_generator()());
                    long currentTime = ros::Time::now().toSec()*1000.0; 
                    request.header.plan_id = planId;
                    request.header.recipient_id = psm_.targetLeaderId;
                    request.header.sender_bsm_id = BSMID;
                    request.header.sender_id = MobilityId;
                    request.header.timestamp = currentTime;
                    cav_msgs::LocationECEF loc;
                    request.location = loc;
                    request.plan_type.type = cav_msgs::PlanType::PLATOON_FOLLOWER_JOIN;
                    std::string MOBILITY_STRATEGY;
                    request.strategy = MOBILITY_STRATEGY;
                    request.strategy_params = "";
                    request.urgency = 50;
                    mobility_request_publisher_(request);
                    // mob_req_pub_.publish(request);
                    ROS_DEBUG("Published Mobility Candidate-Join request to the leader");
                    
                    PlatoonPlan* new_plan = new PlatoonPlan(true, currentTime, planId, psm_.targetLeaderId);

                    psm_.current_plan = *new_plan;
                }
        
         //Task 4
                if(psm_.pm_.getTotalPlatooningSize() > 1) {
                    cav_msgs::MobilityOperation status;
                    composeMobilityOperationCandidateFollower(status);
                    mob_op_pub_.publish(status);
                    ROS_DEBUG("Published platoon STATUS operation message");
                }
                long tsEnd =  ros::Time::now().toSec()*1000;
                long sleepDuration = std::max(int(statusMessageInterval_ - (tsEnd - tsStart)), 0);
                ros::Duration(sleepDuration/1000).sleep();
        
    }



    void PlatoonStrategicPlugin::mob_req_cb(const cav_msgs::MobilityRequest& msg)
    {
        mob_req_cb_leader(msg);
    }

    MobilityRequestResponse PlatoonStrategicPlugin::mob_req_cb_standby(const cav_msgs::MobilityRequest& msg)
    {
        // In standby state, the plugin is not responsible for replying to any request messages
        return MobilityRequestResponse::NO_RESPONSE;
    }

    MobilityRequestResponse PlatoonStrategicPlugin::mob_req_cb_candidatefollower(const cav_msgs::MobilityRequest& msg)
    {
        // This state does not handle any mobility request for now
        // TODO Maybe it should handle some ABORT request from a waiting leader
        ROS_DEBUG("Recived mobility request with type " , msg.plan_type.type , " but ignored.");
        return MobilityRequestResponse::NO_RESPONSE;

    }

    MobilityRequestResponse PlatoonStrategicPlugin::mob_req_cb_follower(const cav_msgs::MobilityRequest& msg)
    {
        return MobilityRequestResponse::NO_RESPONSE;
    }

    MobilityRequestResponse PlatoonStrategicPlugin::mob_req_cb_leaderwaiting(const cav_msgs::MobilityRequest& msg)
    {
        bool isTargetVehicle = (msg.header.sender_id == applicantId_);
        bool isCandidateJoin = msg.plan_type.type == cav_msgs::PlanType::PLATOON_FOLLOWER_JOIN;
        if(isTargetVehicle && isCandidateJoin)
        {
            ROS_DEBUG("Target vehicle " , applicantId_ , " is actually joining.");
            ROS_DEBUG("Changing to PlatoonLeaderState and send ACK to target vehicle");
            // TODO replace this
            // plugin.setState(new LeaderState(plugin, log, pluginServiceLocator));
            return MobilityRequestResponse::ACK;
        }
        else 
        {
            ROS_DEBUG("Received platoon request with vehicle id = " , msg.header.sender_id);
            ROS_DEBUG("The request type is " , msg.plan_type.type , " and we choose to ignore");
            return MobilityRequestResponse::NO_RESPONSE;
        }

    }

    MobilityRequestResponse PlatoonStrategicPlugin::mob_req_cb_leader(const cav_msgs::MobilityRequest& msg)
    {
        mobility_req_msg_ = msg;
        cav_msgs::PlanType plan_type= msg.plan_type;
        if (plan_type.type == cav_msgs::PlanType::JOIN_PLATOON_AT_REAR)
        {
            // We are currently checking two basic JOIN conditions:
            //     1. The size limitation on current platoon based on the plugin's parameters.
            //     2. Calculate how long that vehicle can be in a reasonable distance to actually join us.
            // TODO We ignore the lane information for now and assume the applicant is in the same lane with us.

            cav_msgs::MobilityHeader msgHeader = msg.header;
            std::string params = msg.strategy;
            std::string applicantId = msgHeader.sender_id;
            ROS_DEBUG("Receive mobility JOIN request from " , applicantId , " and PlanId = " , msgHeader.plan_id);
            ROS_DEBUG("The strategy parameters are " , params);
            // For JOIN_PLATOON_AT_REAR message, the strategy params is defined as "SIZE:xx,SPEED:xx,DTD:xx"
            // TODO In future, we should remove down track distance from this string and use location field in request message
            std::vector<std::string> inputsParams;
            boost::algorithm::split(inputsParams, params, boost::is_any_of(","));

            std::vector<std::string> applicantSize_parsed;
            boost::algorithm::split(applicantSize_parsed, inputsParams[0], boost::is_any_of(":"));
            int applicantSize = std::stoi(applicantSize_parsed[1]);
            ROS_DEBUG("applicantSize: " , applicantSize);

            std::vector<std::string> applicantCurrentSpeed_parsed;
            boost::algorithm::split(applicantCurrentSpeed_parsed, inputsParams[1], boost::is_any_of(":"));
            double applicantCurrentSpeed = std::stod(applicantCurrentSpeed_parsed[1]);
            ROS_DEBUG("applicantCurrentSpeed: " , applicantCurrentSpeed);

            std::vector<std::string> applicantCurrentDtd_parsed;
            boost::algorithm::split(applicantCurrentDtd_parsed, inputsParams[2], boost::is_any_of(":"));
            double applicantCurrentDtd = std::stod(applicantCurrentDtd_parsed[1]);
            ROS_DEBUG("applicantCurrentDtd: " , applicantCurrentDtd);

            // Check if we have enough room for that applicant
            int currentPlatoonSize = psm_.pm_.getTotalPlatooningSize();
            bool hasEnoughRoomInPlatoon = applicantSize + currentPlatoonSize <= maxPlatoonSize_;
            if(hasEnoughRoomInPlatoon) {
                ROS_DEBUG("The current platoon has enough room for the applicant with size " , applicantSize);
                double currentRearDtd = psm_.pm_.getPlatoonRearDowntrackDistance();
                ROS_DEBUG("The current platoon rear dtd is " , currentRearDtd);
                double currentGap = currentRearDtd - applicantCurrentDtd - vehicleLength_;
                double currentTimeGap = currentGap / applicantCurrentSpeed;
                ROS_DEBUG("The gap between current platoon rear and applicant is " , currentGap , "m or " , currentTimeGap , "s");
                if(currentGap < 0) {
                    ROS_WARN("We should not receive any request from the vehicle in front of us. NACK it.");
                    return MobilityRequestResponse::NACK;
                }
                // Check if the applicant can join based on max timeGap/gap
                bool isDistanceCloseEnough = (currentGap <= maxAllowedJoinGap_) || (currentTimeGap <= maxAllowedJoinTimeGap_);
                if(isDistanceCloseEnough) {
                    ROS_DEBUG("The applicant is close enough and we will allow it to try to join");
                    ROS_DEBUG("Change to LeaderWaitingState and waiting for " , msg.header.sender_id , " to join");
                    // TODO????????????
                    // plugin.setState(new LeaderWaitingState(plugin, log, pluginServiceLocator, applicantId));??????????????
                    return MobilityRequestResponse::ACK;
                } else {
                    ROS_DEBUG("The applicant is too far away from us. NACK.");
                    return MobilityRequestResponse::NACK;
                }
            }
            else 
            {
                ROS_DEBUG("The current platoon does not have enough room for applicant of size " , applicantSize , ". NACK");
                return MobilityRequestResponse::NACK;
            } 
        } 
        else {
            ROS_DEBUG("Received mobility request with type " , msg.plan_type.type , " and ignored.");
            return MobilityRequestResponse::NO_RESPONSE;
        }
    }

    void PlatoonStrategicPlugin::mob_resp_cb(const cav_msgs::MobilityResponse& msg)
    {
        // In standby state, it will not send out any requests so it will also ignore all responses
    }

    void PlatoonStrategicPlugin::mob_resp_cb_standby(const cav_msgs::MobilityResponse& msg)
    {
        mobility_resp_msg_ = msg;
    }


    void PlatoonStrategicPlugin::mob_resp_cb_candidatefollower(const cav_msgs::MobilityResponse& msg)
    {
        if (psm_.current_plan.valid)
        {
            bool isForCurrentPlan = msg.header.plan_id == psm_.current_plan.planId;
            bool isFromTargetVehicle = msg.header.sender_id == psm_.targetLeaderId;
            if(isForCurrentPlan && isFromTargetVehicle)
            {
                if(msg.is_accepted) {
                    // We change to follower state and start to actually follow that leader
                    // The platoon manager also need to change the platoon Id to the one that the target leader is using 
                    ROS_DEBUG("The leader " , msg.header.sender_id , " agreed on our join. Change to follower state.");
                    // TODO: update these accordingly
                    // plugin.platoonManager.changeFromLeaderToFollower(targetPlatoonId);
                    // plugin.setState(new FollowerState(plugin, log, pluginServiceLocator));
                    // pluginServiceLocator.getArbitratorService().requestNewPlan(this.trajectoryEndLocation);
                } 
                else{
                    // We change back to normal leader state and try to join other platoons
                    ROS_DEBUG("The leader " , msg.header.sender_id , " does not agree on our join. Change back to leader state.");
                    // TODO: update these accordingly
                    // plugin.setState(new LeaderState(plugin, log, pluginServiceLocator));
                }
            }
            else 
            {
                ROS_DEBUG("Ignore received response message because it is not for the current plan.");
            }  
        }
        else 
        {
            ROS_DEBUG("Ignore received response message because we are not in any negotiation process.");
        }
    }

    void PlatoonStrategicPlugin::mob_resp_cb_leaderwaiting(const cav_msgs::MobilityResponse& msg)
    {
    }
    
    void PlatoonStrategicPlugin::mob_resp_cb_follower(const cav_msgs::MobilityResponse& msg)
    {
    }

    void PlatoonStrategicPlugin::mob_resp_cb_leader(const cav_msgs::MobilityResponse& msg)
    {
        if (psm_.current_plan.valid)
        {
            if(psm_.current_plan.planId == msg.header.plan_id && psm_.current_plan.peerId == msg.header.sender_id)
            {
                if (msg.is_accepted)
                {
                    ROS_DEBUG("Received positive response for plan id = " , psm_.current_plan.planId);
                    ROS_DEBUG("Change to CandidateFollower state and notify trajectory failure in order to replan");
                        // Change to candidate follower state and request a new plan to catch up with the front platoon
                        // TODO: update these accordingly
                        // plugin.setState(new CandidateFollowerState(plugin, log, pluginServiceLocator, currentPlan.peerId, potentialNewPlatoonId, this.trajectoryEndLocation));
                        // pluginServiceLocator.getArbitratorService().requestNewPlan(this.trajectoryEndLocation);
                }
                else 
                {
                    ROS_DEBUG("Received negative response for plan id = " , psm_.current_plan.planId);
                    // Forget about the previous plan totally
                    // this.currentPlan = null;
                    psm_.current_plan.valid = false;
                }
            }
            else
            {
                ROS_DEBUG("Ignore the response message because planID match: " , (psm_.current_plan.planId == msg.header.plan_id));
                ROS_DEBUG("My plan id = " , psm_.current_plan.planId , " and response plan Id = " , msg.header.plan_id);
                ROS_DEBUG("And peer id match " , (psm_.current_plan.peerId == msg.header.sender_id));
                ROS_DEBUG("Expected peer id = " , psm_.current_plan.peerId , " and response sender Id = " , msg.header.sender_id);
            }
        }
    }

    void PlatoonStrategicPlugin::mob_op_cb(const cav_msgs::MobilityOperation& msg)
    {
        mob_op_cb_leader(msg);
    }

    void PlatoonStrategicPlugin::mob_op_cb_standby(const cav_msgs::MobilityOperation& msg)
    {
        // In standby state, it will ignore operation message since it is not actively operating
    }

    void PlatoonStrategicPlugin::mob_op_cb_candidatefollower(const cav_msgs::MobilityOperation& msg)
    {
        // We still need to handle STATUS operAtion message from our platoon
        std::string strategyParams = msg.strategy_params;
        bool isPlatoonStatusMsg = (strategyParams.rfind(OPERATION_STATUS_TYPE, 0) == 0);
        if(isPlatoonStatusMsg) {
            std::string vehicleID = msg.header.sender_id;
            std::string platoonId = msg.header.plan_id;
            std::string statusParams = strategyParams.substr(OPERATION_STATUS_TYPE.size() + 1);
            // TODO: update this
            // plugin.platoonManager.memberUpdates(vehicleID, platoonId, msg.getHeader().getSenderBsmId(), statusParams);
            ROS_DEBUG("Received platoon status message from " , msg.header.sender_id);
        }
        else {
            ROS_DEBUG("Received a mobility operation message with params " , msg.strategy_params , " but ignored.");
        }
    }

    void PlatoonStrategicPlugin::mob_op_cb_follower(const cav_msgs::MobilityOperation& msg)
    {
        std::string strategyParams = msg.strategy_params;
        // In the current state, we care about the STATUS message
        bool isPlatoonStatusMsg = (strategyParams.rfind(OPERATION_STATUS_TYPE, 0) == 0);
        bool isPlatoonInfoMsg = (strategyParams.rfind(OPERATION_INFO_TYPE, 0) == 0);
        // If it is platoon status message, the params string is in format:
        // STATUS|CMDSPEED:xx,DTD:xx,SPEED:xx
        if(isPlatoonStatusMsg) {
            std::string vehicleID = msg.header.sender_id;
            std::string platoonID = msg.header.plan_id;
            std::string statusParams = strategyParams.substr(OPERATION_STATUS_TYPE.size() + 1);
            ROS_DEBUG("Receive operation message from vehicle: " , vehicleID);
            // TODO: update this
            // plugin.platoonManager.memberUpdates(vehicleID, platoonID, msg.getHeader().getSenderBsmId(), statusParams);
        }
    }

    void PlatoonStrategicPlugin::mob_op_cb_leaderwaiting(const cav_msgs::MobilityOperation& msg)
    {
        std::string strategyParams = msg.strategy_params;
        bool isPlatoonStatusMsg = (strategyParams.rfind(OPERATION_STATUS_TYPE, 0) == 0);
        if(isPlatoonStatusMsg) {
            std::string vehicleID = msg.header.sender_id;
            std::string platoonId = msg.header.plan_id;
            std::string statusParams = strategyParams.substr(OPERATION_STATUS_TYPE.size() + 1);
            // / TODO: update these accordingly
            // plugin.platoonManager.memberUpdates(vehicleID, platoonId, msg.getHeader().getSenderBsmId(), statusParams);
            ROS_DEBUG("Received platoon status message from " , msg.header.sender_id);
        } else {
            ROS_DEBUG("Received a mobility operation message with params " , msg.strategy_params , " but ignored.");
        }
    }

    void PlatoonStrategicPlugin::mob_op_cb_leader(const cav_msgs::MobilityOperation& msg)
    {

        std::string strategyParams = msg.strategy_params;
        std::string senderId = msg.header.sender_id;
        std::string platoonId = msg.header.plan_id;
        // In the current state, we care about the INFO heart-beat operation message if we are not currently in
        // a negotiation, and also we need to care about operation from members in our current platoon

        bool isPlatoonInfoMsg = (strategyParams.rfind(OPERATION_INFO_TYPE, 0) == 0);
        bool isPlatoonStatusMsg = (strategyParams.rfind(OPERATION_STATUS_TYPE, 0) == 0);
        // original: boolean isNotInNegotiation = (this.currentPlan == null);
        bool isNotInNegotiation = psm_.current_plan.valid;
        if(isPlatoonInfoMsg && isNotInNegotiation)
        {
            // For INFO params, the string format is INFO|REAR:%s,LENGTH:%.2f,SPEED:%.2f,SIZE:%d
            // TODO In future, we should remove downtrack distance from this string and send XYZ location in ECEF

            std::vector<std::string> inputsParams;
            boost::algorithm::split(inputsParams, strategyParams, boost::is_any_of(","));

            std::vector<std::string> rearVehicleBsmId_parsed;
            boost::algorithm::split(rearVehicleBsmId_parsed, inputsParams[0], boost::is_any_of(":"));
            std::string rearVehicleBsmId = rearVehicleBsmId_parsed[1];
            ROS_DEBUG("rearVehicleBsmId: " , rearVehicleBsmId);

            std::vector<std::string> rearVehicleDtd_parsed;
            boost::algorithm::split(rearVehicleDtd_parsed, inputsParams[0], boost::is_any_of(":"));
            double rearVehicleDtd = std::stod(rearVehicleDtd_parsed[4]);
            ROS_DEBUG("rearVehicleDtd: " , rearVehicleDtd);
            // We are trying to validate is the platoon rear is right in front of the host vehicle
            if(isVehicleRightInFront(rearVehicleBsmId, rearVehicleDtd))
            {
                ROS_DEBUG("Found a platoon with id = " , platoonId , " in front of us.");
                cav_msgs::MobilityRequest request;
                request.header.plan_id = boost::uuids::to_string(boost::uuids::random_generator()());
                request.header.recipient_id = senderId;
                request.header.sender_bsm_id = host_bsm_id_;
                request.header.sender_id = HostMobilityId;
                request.header.timestamp = ros::Time::now().toNSec() * 1000000;
                // request.location  TODO: add ecef location here
                request.plan_type.type = cav_msgs::PlanType::JOIN_PLATOON_AT_REAR;
                request.strategy = MOBILITY_STRATEGY;

                int platoon_size = psm_.pm_.getTotalPlatooningSize();
                double current_speed = psm_.pm_.getCurrentSpeed();
                double current_downtrack;///TODO: replace with downtrack update function

                boost::format fmter(OPERATION_STATUS_PARAMS);
                fmter %platoon_size;
                fmter %current_speed;
                fmter %current_downtrack;
                
                request.strategy_params = fmter.str();
                request.urgency = 50;

                // this.currentPlan = new PlatoonPlan(System.currentTimeMillis(), request.getHeader().getPlanId(), senderId);
                mob_op_pub_.publish(request);
                ROS_DEBUG("Publishing request to leader " , senderId , " with params " , request.strategy_params , " and plan id = " , request.header.plan_id);
                // this.potentialNewPlatoonId = platoonId;
            }
            else 
            {
                ROS_DEBUG("Ignore platoon with platoon id: " , platoonId , " because it is not right in front of us");
            }
        }
        else if(isPlatoonStatusMsg) 
        {
            // If it is platoon status message, the params string is in format: STATUS|CMDSPEED:xx,DTD:xx,SPEED:xx
            std::string statusParams = strategyParams.substr(OPERATION_STATUS_TYPE.length() + 1);
            ROS_DEBUG("Receive operation status message from vehicle: " , senderId , " with params: " , statusParams);
            psm_.pm_.memberUpdates(senderId, platoonId, msg.header.sender_bsm_id, statusParams);

        }
        else
        {
            ROS_DEBUG("Receive operation message but ignore it because isPlatoonInfoMsg = " , isPlatoonInfoMsg, 
            ", isNotInNegotiation = " , isNotInNegotiation , " and isPlatoonStatusMsg = " , isPlatoonStatusMsg);
        }
    }

    bool PlatoonStrategicPlugin::isVehicleRightInFront(std::string rearVehicleBsmId, double downtrack) {
        // double currentDtd = getCurrentDowntrackDistance();
        double currentDtd;
        if(downtrack > currentDtd) {
            ROS_DEBUG("Found a platoon in front. We are able to join");
            return true;
        } else {
            ROS_DEBUG("Ignoring platoon from our back.");
            ROS_DEBUG("The front platoon dtd is " , downtrack , " and we are current at " , currentDtd);
            return false;
        }
    }


    void PlatoonStrategicPlugin::composeMobilityOperationLeader(cav_msgs::MobilityOperation &msg, const std::string& type){
        msg.header.plan_id = psm_.pm_.currentPlatoonID;
        msg.header.recipient_id = "";
        msg.header.sender_bsm_id = BSMID;
        std::string hostStaticId = HostMobilityId;
        msg.header.sender_id = hostStaticId;
        msg.header.timestamp = ros::Time::now().toSec()*1000.0;
        msg.strategy = MOBILITY_STRATEGY;

        if (type == OPERATION_INFO_TYPE){
            // For INFO params, the string format is INFO|REAR:%s,LENGTH:%.2f,SPEED:%.2f,SIZE:%d

            std::string PlatoonRearBsmId = BSMID;
            int CurrentPlatoonLength = psm_.pm_.getCurrentPlatoonLength();
            double current_speed = psm_.pm_.getCurrentSpeed();
            int PlatoonSize = psm_.pm_.getTotalPlatooningSize();
            double PlatoonRearDowntrackDistance = psm_.pm_.getPlatoonRearDowntrackDistance();

            boost::format fmter(OPERATION_INFO_TYPE);
            fmter %PlatoonRearBsmId;
            fmter %CurrentPlatoonLength;
            fmter %current_speed;
            fmter %PlatoonSize;
            fmter %PlatoonRearDowntrackDistance;

            std::string infoParams = fmter.str();
            msg.strategy_params = infoParams;
        }
        else if (type == OPERATION_STATUS_TYPE){
            // For STATUS params, the string format is "STATUS|CMDSPEED:xx,DTD:xx,SPEED:xx"
            double cmdSpeed, current_speed, current_downtrack;
            boost::format fmter(OPERATION_STATUS_PARAMS);
            fmter %cmdSpeed;
            fmter %current_downtrack;
            fmter %current_speed;
                    
            std::string statusParams = fmter.str();
            msg.strategy_params = statusParams;
        } else {
            ROS_ERROR("UNKNOW strategy param string!!!");
            msg.strategy_params = "";
        }
        ROS_DEBUG("Composed a mobility operation message with params " , msg.strategy_params);
    
    }

    void PlatoonStrategicPlugin::composeMobilityOperationFollower(cav_msgs::MobilityOperation &msg) const{
        msg.header.plan_id = psm_.pm_.currentPlatoonID;
        // All platoon mobility operation message is just for broadcast
        msg.header.recipient_id = "";
        msg.header.sender_bsm_id = BSMID;
        std::string hostStaticId = HostMobilityId;
        msg.header.sender_id = hostStaticId;
        msg.header.timestamp = ros::Time::now().toSec()*1000.0;
        msg.strategy = MOBILITY_STRATEGY;
        
        double cmdSpeed, current_speed, current_downtrack;

        boost::format fmter(OPERATION_STATUS_PARAMS);
        fmter %cmdSpeed;
        fmter %current_downtrack;
        fmter %current_speed;
                    
        std::string statusParams = fmter.str();
        msg.strategy_params = statusParams;
        ROS_DEBUG("Composed a mobility operation message with params " , msg.strategy_params);
    }


    void PlatoonStrategicPlugin::composeMobilityOperationLeaderWaiting(cav_msgs::MobilityOperation &msg) const
    {
        msg.header.plan_id = psm_.pm_.currentPlatoonID;
        // This message is for broadcast
        msg.header.recipient_id = "";
        msg.header.sender_bsm_id = BSMID;
        std::string hostStaticId = HostMobilityId;
        msg.header.sender_id = hostStaticId;
        msg.header.timestamp = ros::Time::now().toSec()*1000;
        msg.strategy = MOBILITY_STRATEGY;
        // For STATUS params, the string format is "STATUS|CMDSPEED:5.0,DOWNTRACK:100.0,SPEED:5.0"
        double cmdSpeed, current_speed, current_downtrack;
        boost::format fmter(OPERATION_STATUS_PARAMS);
        fmter %cmdSpeed;
        fmter %current_downtrack;
        fmter %current_speed;
                    
        std::string statusParams = fmter.str();
        msg.strategy_params = statusParams;
        
    }


    void PlatoonStrategicPlugin::composeMobilityOperationCandidateFollower(cav_msgs::MobilityOperation &msg)
    {
        msg.header.plan_id = psm_.pm_.currentPlatoonID;
        // All platoon mobility operation message is just for broadcast
        msg.header.recipient_id = "";
        msg.header.sender_bsm_id = BSMID;
        std::string hostStaticId = HostMobilityId;
        msg.header.sender_id = hostStaticId;
        msg.header.timestamp = ros::Time::now().toSec()*1000.0; 
        msg.strategy = MOBILITY_STRATEGY;
        
        // // For STATUS params, the string format is "STATUS|CMDSPEED:xx,DTD:xx,SPEED:xx"
        
        double cmdSpeed, current_speed, current_downtrack;
        boost::format fmter(OPERATION_STATUS_PARAMS);
        fmter %cmdSpeed;
        fmter %current_downtrack;
        fmter %current_speed;
                    
        std::string statusParams = fmter.str();
        msg.strategy_params = statusParams;
        ROS_DEBUG("Composed a mobility operation message with params " , msg.strategy_params);
    }


}
