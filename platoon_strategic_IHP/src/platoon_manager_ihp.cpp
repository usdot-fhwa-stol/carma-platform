/*
 * Copyright (C) 2019-2021 LEIDOS.
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

/*
 * Developed by the UCLA Mobility Lab, 10/20/2021. 
 *
 * Creator: Xu Han
 * Author: Xu Han, Xin Xia, Jiaqi Ma
 */

#include "platoon_strategic_ihp/platoon_manager_ihp.h"
#include "platoon_strategic_ihp/platoon_config_ihp.h"
#include <boost/algorithm/string.hpp>
#include <rclcpp/logging.hpp>
#include <array>

namespace platoon_strategic_ihp
{
    /**
     * Implementation notes:  
     * 
     * 1. platoon vector indexing: 
     *  A vector of platoon members (vehicles), sorted by downtrack distance in descending order (i.e., [dtd_1 > dtd_2 > ... > dtd_n] ). 
     * 
     * 2. speed vector indexing: 
     *  A vector that only contains speed (m/s) of each platoon member. Same order with platoon list (i.e., [platoon_front, follwer_1, ..., follower_n]).
     * 
     * 3. downtrackDistance vector indexing:
     *  A vector that only contains downtrack distance (m) of each platoon member. Same order with platoon list (i.e., [platoon_front, follwer_1, ..., follower_n])
     * 
     * 4. timeHeadway vector indexing 
     *  A vector that only contains time headaway (s) behind each platoon member (i.e., the time gap between host vehicle and its following vehicle). 
     *  Same order with platoon list. For APF, when gap too small, the dynamic leader will be the front vehicle of the small gap. If gap too large, the dynamic leader 
     *  wil be the rear vehicle of the large gap.
     * 
     */

    PlatoonManager::PlatoonManager(std::shared_ptr<carma_ros2_utils::timers::TimerFactory> timer_factory) : timer_factory_(std::move(timer_factory))
    {
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_strategic_ihp"), "Top of PlatoonManager ctor.");
    }


    // Update the location of the host in the vector of platoon members
    void PlatoonManager::updateHostPose(const double downtrack, const double crosstrack)
    {
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_strategic_ihp"), "Host (index " << hostPosInPlatoon_ << "): downtrack = " << downtrack << ", crosstrack = " << crosstrack);
        host_platoon_[hostPosInPlatoon_].vehiclePosition = downtrack;
        host_platoon_[hostPosInPlatoon_].vehicleCrossTrack = crosstrack;
    }

    // Update the speed info of the host in the vector of platoon members
    void PlatoonManager::updateHostSpeeds(const double cmdSpeed, const double actualSpeed)
    {
        host_platoon_[hostPosInPlatoon_].commandSpeed = cmdSpeed;
        host_platoon_[hostPosInPlatoon_].vehicleSpeed = actualSpeed;
    }

    // Update/add one member's information from STATUS messages, update platoon ID if needed.  
    void PlatoonManager::hostMemberUpdates(const std::string& senderId, const std::string& platoonId, const std::string& params, 
                                           const double& DtD, const double& CtD)
    {

        // parse params, read member data
        std::vector<std::string> inputsParams;
        boost::algorithm::split(inputsParams, params, boost::is_any_of(","));
        // read command speed, m/s
        std::vector<std::string> cmd_parsed;
        boost::algorithm::split(cmd_parsed, inputsParams[0], boost::is_any_of(":"));
        double cmdSpeed = std::stod(cmd_parsed[1]);
        // get DtD directly instead of parsing message, m
        double dtDistance = DtD;
        // get CtD directly 
        double ctDistance = CtD;
        // read current speed, m/s
        std::vector<std::string> cur_parsed;
        boost::algorithm::split(cur_parsed, inputsParams[1], boost::is_any_of(":"));
        double curSpeed = std::stod(cur_parsed[1]);

        // If we are currently in a follower state:
        // 1. We will update platoon ID based on leader's STATUS
        // 2. We will update platoon members info based on platoon ID for all members
        if (isFollower) 
        {
            // Sanity check
            if (platoonLeaderID.compare(host_platoon_[0].staticId) != 0)
            {
                RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_strategic_ihp"), "///// platoonLeaderID NOT PROPERLY ASSIGNED! Value = " << platoonLeaderID
                                << ", host_platoon_[0].staticId = " << host_platoon_[0].staticId);
            }

            // read message status        
            bool isFromLeader = platoonLeaderID == senderId;
            bool needPlatoonIdChange = isFromLeader && (currentPlatoonID != platoonId);

            if(needPlatoonIdChange)
            {
                // TODO: better to have leader explicitly inform us that it is merging into another platoon, rather than require us to deduce it here.
                //       This condition may also result from missing some message traffic, new leader in this platoon, or other confusion/incorrect code.
                //       Consider using a new STATUS msg param that tells members of new platoon ID and new leader ID when a change is made.
                RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_strategic_ihp"), "It seems that the current leader is joining another platoon.");
                RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_strategic_ihp"), "So the platoon ID is changed from " << currentPlatoonID << " to " << platoonId);
                currentPlatoonID = platoonId;
                updatesOrAddMemberInfo(host_platoon_, senderId, cmdSpeed, dtDistance, ctDistance, curSpeed); 
            } 
            else if (currentPlatoonID == platoonId)
            {
                RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_strategic_ihp"), "This STATUS messages is from our platoon. Updating the info...");
                updatesOrAddMemberInfo(host_platoon_, senderId, cmdSpeed, dtDistance, ctDistance, curSpeed);
                RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_strategic_ihp"), "The first vehicle in our list is now " << host_platoon_[0].staticId);
            } 
            else //sender is in a different platoon
            {
                RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_strategic_ihp"), "This STATUS message is not from a vehicle we care about. Ignore this message with id: " << senderId);
            }
        }
        else //host is leader
        {
            // If we are currently in any leader state, we only update platoon member based on platoon ID
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_strategic_ihp"), "Host is leader: currentPlatoonID = " << currentPlatoonID << ", incoming platoonId = " << platoonId);
            if (currentPlatoonID == platoonId)
            {
                RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_strategic_ihp"), "This STATUS messages is from our platoon. Updating the info...");
                updatesOrAddMemberInfo(host_platoon_, senderId, cmdSpeed, dtDistance, ctDistance, curSpeed); 
            }
            else
            {
                RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_strategic_ihp"), "Platoon IDs not matched");
                RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_strategic_ihp"), "currentPlatoonID: " << currentPlatoonID);
                RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_strategic_ihp"), "incoming platoonId: " << platoonId);
            }
        }
        
        // update host vehicle information each time new member is updated. Now platoon contains host vehicle.
        std::string hostStaticId = getHostStaticID();
        double hostcmdSpeed = getCommandSpeed();
        double hostDtD = getCurrentDowntrackDistance();
        double hostCtD = getCurrentCrosstrackDistance();
        double hostCurSpeed = getCurrentSpeed();
        updatesOrAddMemberInfo(host_platoon_, hostStaticId, hostcmdSpeed, hostDtD, hostCtD, hostCurSpeed);
    }
    
    // Update/add one member's information from STATUS messages, update platoon ID if needed.  
    void PlatoonManager::neighborMemberUpdates(const std::string& senderId, const std::string& platoonId, const std::string& params, 
                                               const double& DtD, const double& CtD)
    {

        // parse params, read member data
        std::vector<std::string> inputsParams;
        boost::algorithm::split(inputsParams, params, boost::is_any_of(","));
        // read command speed, m/s
        std::vector<std::string> cmd_parsed;
        boost::algorithm::split(cmd_parsed, inputsParams[0], boost::is_any_of(":"));
        double cmdSpeed = std::stod(cmd_parsed[1]);
        // get DtD directly instead of parsing message, m
        double dtDistance = DtD;
        // get CtD directly 
        double ctDistance = CtD;
        // read current speed, m/s
        std::vector<std::string> cur_parsed;
        boost::algorithm::split(cur_parsed, inputsParams[1], boost::is_any_of(":"));
        double curSpeed = std::stod(cur_parsed[1]);

        if (neighborPlatoonID == platoonId)
        {
            updatesOrAddMemberInfo(neighbor_platoon_, senderId, cmdSpeed, dtDistance, ctDistance, curSpeed);
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_strategic_ihp"), "This STATUS messages is from the target platoon. Updating the info...");
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_strategic_ihp"), "The first vehicle in that platoon is now " << neighbor_platoon_[0].staticId);

            // If we have data on all members of a neighboring platoon, set a complete record flag
            if (neighbor_platoon_info_size_ > 1  &&
                neighbor_platoon_.size() == neighbor_platoon_info_size_)
            {
                is_neighbor_record_complete_ = true;
                RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_strategic_ihp"), "Neighbor record is complete!");
            }
        } 
        else //sender is in a different platoon
        {
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_strategic_ihp"), "This STATUS message is not from a vehicle we care about. Ignore this message from sender: " 
                            << senderId << " about platoon: " << platoonId);
        }
    }
    
    // Check a new vehicle's existence; add its info to the platoon if not in list, update info if already existed. 
    void PlatoonManager::updatesOrAddMemberInfo(std::vector<PlatoonMember>& platoon, std::string senderId, double cmdSpeed,
                                                double dtDistance, double ctDistance, double curSpeed)
    {
        bool isExisted = false;
        bool sortNeeded = false;

        // update/add this info into the list
        for (size_t i = 0;  i < platoon.size();  ++i){
            if(platoon[i].staticId == senderId) {
                if (abs(dtDistance - platoon[i].vehiclePosition)/(platoon[i].vehiclePosition + 0.01) > config_.significantDTDchange)
                {
                    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_strategic_ihp"),  "DTD of member " << platoon[i].staticId << " is changed significantly, so a new sort is needed");

                    sortNeeded = true;
                }
                platoon[i].commandSpeed = cmdSpeed;         // m/s
                platoon[i].vehiclePosition = dtDistance;    // m 
                platoon[i].vehicleCrossTrack = ctDistance;  // m
                platoon[i].vehicleSpeed = curSpeed;         // m/s
                platoon[i].timestamp = timer_factory_->now().nanoseconds()/1000000;
                RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_strategic_ihp"), "Receive and update platooning info on member " << i << ", ID:" << platoon[i].staticId);
                RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_strategic_ihp"), "    CommandSpeed       = " << platoon[i].commandSpeed);
                RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_strategic_ihp"), "    Actual Speed       = " << platoon[i].vehicleSpeed);
                RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_strategic_ihp"), "    Downtrack Location = " << platoon[i].vehiclePosition);
                RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_strategic_ihp"), "    Crosstrack dist    = " << platoon[i].vehicleCrossTrack);

                if (senderId == HostMobilityId)
                {
                    hostPosInPlatoon_ = i;
                    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_strategic_ihp"), "    This is the HOST vehicle");
                }
                isExisted = true;
            }
        }

        if (sortNeeded)
        {
            // sort the platoon member based on dowtrack distance (m) in an descending order.
            std::sort(std::begin(platoon), std::end(platoon), [](const PlatoonMember &a, const PlatoonMember &b){return a.vehiclePosition > b.vehiclePosition;});
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_strategic_ihp"), "Platoon is re-sorted due to large difference in dtd update.");
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_strategic_ihp"), "    Platoon order is now:");
            for (size_t i = 0;  i < platoon.size();  ++i)
            {
                std::string hostFlag = " ";
                if (platoon[i].staticId == getHostStaticID())
                {
                    hostPosInPlatoon_ = i;
                    hostFlag = "Host";
                }
                RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_strategic_ihp"), "    " << platoon[i].staticId << "its DTD: " << platoon[i].vehiclePosition << " " << hostFlag);
            }
        }

        // if not already exist, add to platoon list.
        if(!isExisted) {
            long cur_t = timer_factory_->now().nanoseconds()/1000000; // time in millisecond

            PlatoonMember newMember = PlatoonMember(senderId, cmdSpeed, curSpeed, dtDistance, ctDistance, cur_t);
            platoon.push_back(newMember);
            // sort the platoon member based on downtrack distance (m) in an descending order.
            std::sort(std::begin(platoon), std::end(platoon), [](const PlatoonMember &a, const PlatoonMember &b){return a.vehiclePosition > b.vehiclePosition;});

            RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_strategic_ihp"), "Add a new vehicle into our platoon list " << newMember.staticId << " platoon.size now = " << platoon.size());
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_strategic_ihp"), "    Platoon order is now:");
            for (size_t i = 0;  i < platoon.size();  ++i)
            {
                std::string hostFlag = " ";
                if (platoon[i].staticId == getHostStaticID())
                {
                    hostPosInPlatoon_ = i;
                    hostFlag = "Host";
                }
                RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_strategic_ihp"), "    " << platoon[i].staticId << "its DTD: " << platoon[i].vehiclePosition << " " << hostFlag);
            }
        }
    }
    
    // TODO: Place holder for delete member info due to dissolve operation.

    // Get the platoon size.
    int PlatoonManager::getHostPlatoonSize() {
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_strategic_ihp"), "host platoon size: " << host_platoon_.size());
        return host_platoon_.size();
    }

    // Reset variables to indicate there is no current action in work
    void PlatoonManager::clearActionPlan()
    {
        current_plan.valid = false;
        current_plan.planId = dummyID;
        current_plan.peerId = dummyID;
        targetPlatoonID = dummyID;
        // Leave the platoon & leader IDs alone since we might continue to be in one
    }

    // Reset variables to indicate there is no platoon - host is a solo vehicle again
    void PlatoonManager::resetHostPlatoon()
    {
        // Remove any elements in the platoon vector other than the host vehicle
        if (host_platoon_.size() > hostPosInPlatoon_ + 1)
        {
            host_platoon_.erase(host_platoon_.begin() + hostPosInPlatoon_ + 1, host_platoon_.end());
        }
        host_platoon_.erase(host_platoon_.begin(), host_platoon_.begin() + hostPosInPlatoon_);

        // Clean up other variables
        currentPlatoonID = dummyID;
        platoonLeaderID = dummyID;
        hostPosInPlatoon_ = 0;
        isCreateGap = false;
        dynamic_leader_index_ = 0;
    }

    // Reset variables to indicate there is no known neighbor platoon
    void PlatoonManager::resetNeighborPlatoon()
    {
        neighbor_platoon_.clear();
        neighbor_platoon_info_size_ = 0;
        neighborPlatoonID = dummyID;
        neighbor_platoon_leader_id_ = dummyID;
        is_neighbor_record_complete_ = false;
    }

    bool PlatoonManager::removeMember(const size_t mem)
    {
        // Don't remove ourselves!
        if (hostPosInPlatoon_ == mem)
        {
            return false;
        }

        // Don't remove a member that isn't there
        else if (host_platoon_.size() <= 1  ||  mem >= host_platoon_.size())
        {
            return false;
        }

        if (mem < hostPosInPlatoon_)
        {
            --hostPosInPlatoon_;
        }
        host_platoon_.erase(host_platoon_.begin() + mem, host_platoon_.begin() + mem + 1);

        // If host is the only remaining member then clean up the other platoon data
        if (host_platoon_.size() == 1)
        {
            currentPlatoonID = dummyID;
            platoonLeaderID = dummyID;
            hostPosInPlatoon_ = 0;
        }

        return true;
    }

    bool PlatoonManager::removeMemberById(const std::string id)
    {
        // Don't remove ourselves!
        if (id.compare(HostMobilityId) == 0)
        {
            return false;
        }

        // Search for the member with a matching ID and remove it
        for (size_t m = 0;  m < host_platoon_.size();  ++m)
        {
            if (id.compare(host_platoon_[m].staticId) == 0)
            {
                return removeMember(m);
            }
        }

        // Indicate the member was not found
        return false;
    }
        
    // Find the downtrack distance of the last vehicle of the platoon, in m.    
    double PlatoonManager::getPlatoonRearDowntrackDistance(){
        // due to downtrack descending order, the 1ast vehicle in list is the platoon rear vehicle.
        // Even if host is solo, platoon size is 1 so this works.
        return host_platoon_[host_platoon_.size()-1].vehiclePosition;
    }

    // Find the downtrack distance of the first vehicle of the platoon, in m.
    double PlatoonManager::getPlatoonFrontDowntrackDistance(){
        // due to downtrack descending order, the firest vehicle in list is the platoon front vehicle. 
        return host_platoon_[0].vehiclePosition;
    }

    // Return the dynamic leader (i.e., the vehicle to follow) of the host vehicle.
    PlatoonMember PlatoonManager::getDynamicLeader(){
        PlatoonMember dynamicLeader;
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_strategic_ihp"), "host_platoon_ size: " << host_platoon_.size());
        if(isFollower) 
        {
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_strategic_ihp"), "Leader initially set as first vehicle in platoon");
            // return the first vehicle in the platoon as default if no valid algorithm applied
            // due to downtrack descending order, the platoon front veihcle is the first in list. 
            dynamicLeader = host_platoon_[0];
            if (algorithmType_ == "APF_ALGORITHM"){
                size_t newLeaderIndex = allPredecessorFollowing();

                dynamic_leader_index_ = (int)newLeaderIndex;
                RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_strategic_ihp"), "dynamic_leader_index_: " << dynamic_leader_index_);
                if(newLeaderIndex < host_platoon_.size()) { //this must always be true!
                    dynamicLeader = host_platoon_[newLeaderIndex];
                    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_strategic_ihp"), "APF output: " << dynamicLeader.staticId);
                    previousFunctionalDynamicLeaderIndex_ = newLeaderIndex;
                    previousFunctionalDynamicLeaderID_ = dynamicLeader.staticId;
                }
                else //something is terribly wrong in the logic!
                {
                    RCLCPP_WARN_STREAM(rclcpp::get_logger("platoon_strategic_ihp"), "newLeaderIndex = " << newLeaderIndex << " is invalid coming from allPredecessorFollowing!");
                    /**
                     * it might happened when the subject vehicle gets far away from the preceding vehicle, 
                     * in which case the host vehicle will follow the one in front.
                     */
                    dynamicLeader = host_platoon_[getNumberOfVehicleInFront() - 1];
                    // update index and ID 
                    previousFunctionalDynamicLeaderIndex_ = getNumberOfVehicleInFront()-1;
                    previousFunctionalDynamicLeaderID_ = dynamicLeader.staticId;
                    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_strategic_ihp"), "Based on the output of APF algorithm we start to follow our predecessor.");
                }
            }
        }
        return dynamicLeader;

    }

    // The implementation of all predecessor following algorithm. Determine the dynamic leader for the host vehicle to follow.
    int PlatoonManager::allPredecessorFollowing(){

        ///***** Case Zero *****///
        // If the host vehicle is the second follower of a platoon, it will always follow the platoon leader in the front 
        if(getNumberOfVehicleInFront() == 1)
        {
            // If the host is the second vehicle, then follow the leader.
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_strategic_ihp"), "As the second vehicle in the platoon, it will always follow the leader. Case Zero");
            return 0;
        }
        ///***** Case One *****///
        // If there weren't a leader in the previous time step, follow the first vehicle (i.e., the platoon leader) as default.
        if(previousFunctionalDynamicLeaderID_ == "") {
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_strategic_ihp"), "APF algorithm did not found a dynamic leader in previous time step. Case one");
            return 0;
        }

        //***** Formulate speed and downtrack vector *****//
        // Update host vehicle info when update member info, so platoon list include host vehicle, direct use platoon size for downtrack/speed vector.
        // Record downtrack distance (m) of each member
        std::vector<double> downtrackDistance(hostPosInPlatoon_);
        for(size_t i = 0; i < hostPosInPlatoon_; i++) {
            downtrackDistance[i] = host_platoon_[i].vehiclePosition; // m
        }
        // Record speed (m/s) of each member
        std::vector<double> speed(host_platoon_.size());
        for(size_t i = 0; i < host_platoon_.size(); i++) {
            speed[i] = host_platoon_[i].vehicleSpeed; // m/s
        }
        

        ///***** Case Two *****///
        // If the distance headway between the subject vehicle and its predecessor is an issue
        // according to the "min_gap" and "max_gap" thresholds, then it should follow its predecessor
        // The following line will not throw exception because the length of downtrack array is larger than two in this case
        double distHeadwayWithPredecessor = downtrackDistance[hostPosInPlatoon_ - 1] - downtrackDistance[hostPosInPlatoon_];
        gapWithPred_ = distHeadwayWithPredecessor;
        if(insufficientGapWithPredecessor(distHeadwayWithPredecessor)) {
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_strategic_ihp"), "APF algorithm decides there is an issue with the gap with preceding vehicle: " << distHeadwayWithPredecessor << " m. Case Two");
            return hostPosInPlatoon_ - 1;
        }
        else{
            // implementation of the main part of APF algorithm
            // calculate the time headway between every consecutive pair of vehicles
            std::vector<double> timeHeadways = calculateTimeHeadway(downtrackDistance, speed);
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_strategic_ihp"), "APF calculate time headways: " );
            for (const auto& value : timeHeadways)
            {
                RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_strategic_ihp"), "APF time headways: " << value);
            }
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_strategic_ihp"), "APF found the previous dynamic leader is " << previousFunctionalDynamicLeaderID_);
            // if the previous dynamic leader is the first vehicle in the platoon
            
            if(previousFunctionalDynamicLeaderIndex_ == 0) {

                ///***** Case Three *****///
                // If there is a violation, the return value is the desired dynamic leader index
                RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_strategic_ihp"), "APF use violations on lower boundary or maximum spacing to choose dynamic leader. Case Three.");
                return determineDynamicLeaderBasedOnViolation(timeHeadways);
            }
            else{
                // if the previous dynamic leader is not the first vehicle
                // get the time headway between every consecutive pair of vehicles from index Of Previous dynamic Leader
                std::vector<double> partialTimeHeadways = getTimeHeadwayFromIndex(timeHeadways, previousFunctionalDynamicLeaderIndex_);
                for (const auto& value : partialTimeHeadways)
                {
                    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_strategic_ihp"), "APF partial time headways: " << value);
                }
                int closestLowerBoundaryViolation = findLowerBoundaryViolationClosestToTheHostVehicle(partialTimeHeadways);
                int closestMaximumSpacingViolation = findMaximumSpacingViolationClosestToTheHostVehicle(partialTimeHeadways);
                // if there are no violations anywhere between the subject vehicle and the current dynamic leader,
                // then depending on the time headways of the ENTIRE platoon, the subject vehicle may switch
                // dynamic leader further downstream. This is because the subject vehicle has determined that there are
                // no time headways between itself and the current dynamic leader which would cause the platoon to be unsafe.
                // if there are violations somewhere betweent the subject vehicle and the current dynamic leader,
                // then rather than assigning dynamic leadership further DOWNSTREAM, we must go further UPSTREAM in the following lines
                if(closestLowerBoundaryViolation == -1 && closestMaximumSpacingViolation == -1) {
                    // In order for the subject vehicle to assign dynamic leadership further downstream,
                    // two criteria must be satisfied: first the leading vehicle and its immediate follower must
                    // have a time headway greater than "upper_boundary." The purpose of this criteria is to
                    // introduce a hysteresis in order to eliminate the possibility of a vehicle continually switching back 
                    // and forth between two dynamic leaders because one of the time headways is hovering right around
                    // the "lower_boundary" threshold; second the leading vehicle and its predecessor must have
                    // a time headway less than "min_spacing" second. Just as with "upper_boundary", "min_spacing" exists to
                    // introduce a hysteresis where dynamic leaders are continually being switched.
                    bool condition1 = timeHeadways[previousFunctionalDynamicLeaderIndex_] > config_.headawayStableLowerBond; 
                    bool condition2 = timeHeadways[previousFunctionalDynamicLeaderIndex_ - 1] < config_.headawayStableUpperBond; 
                    
                    ///***** Case Four *****///
                    //we may switch dynamic leader further downstream
                    if(condition1 && condition2) {
                        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_strategic_ihp"), "APF found two conditions for assigning local dynamic leadership further downstream are satisfied. Case Four");
                        return determineDynamicLeaderBasedOnViolation(timeHeadways);
                    } else {
                        
                        ///***** Case Five *****///
                        // We may not switch dynamic leadership to another vehicle further downstream because some criteria are not satisfied
                        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_strategic_ihp"), "APF found two conditions for assigning local dynamic leadership further downstream are not satisfied. Case Five.");
                        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_strategic_ihp"), "condition1: " << condition1 << " & condition2: " << condition2);
                        return previousFunctionalDynamicLeaderIndex_;
                    }
                } else if(closestLowerBoundaryViolation != -1 && closestMaximumSpacingViolation == -1) {
                    // The rest four cases have roughly the same logic: locate the closest violation and assign dynamic leadership accordingly
                    
                    ///***** Case Six *****///
                    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_strategic_ihp"), "APF found closestLowerBoundaryViolation on partial time headways. Case Six.");
                    return previousFunctionalDynamicLeaderIndex_ - 1 + closestLowerBoundaryViolation;

                } else if(closestLowerBoundaryViolation == -1 && closestMaximumSpacingViolation != -1) {
                    
                    ///***** Case Seven *****///
                    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_strategic_ihp"), "APF found closestMaximumSpacingViolation on partial time headways. Case Seven.");
                    return previousFunctionalDynamicLeaderIndex_ + closestMaximumSpacingViolation;
                } else{
                    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_strategic_ihp"), "APF found closestMaximumSpacingViolation and closestLowerBoundaryViolation on partial time headways.");
                    if(closestLowerBoundaryViolation > closestMaximumSpacingViolation) {
                        
                        ///***** Case Eight *****///
                        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_strategic_ihp"), "closest LowerBoundaryViolation is higher than closestMaximumSpacingViolation on partial time headways. Case Eight.");
                        return previousFunctionalDynamicLeaderIndex_ - 1 + closestLowerBoundaryViolation;
                    } else if(closestLowerBoundaryViolation < closestMaximumSpacingViolation) {
                        
                        ///***** Case Nine *****///
                        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_strategic_ihp"), "closestMaximumSpacingViolation is higher than closestLowerBoundaryViolation on partial time headways. Case Nine.");
                        return previousFunctionalDynamicLeaderIndex_ + closestMaximumSpacingViolation;
                    } else {
                        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_strategic_ihp"), "APF dynamic Leader selection parameter is wrong!");
                        return 0;
                    }
                }
            }

        }
    }

    // Find the time headaway (s) sub-list based on the platoon wise comprehensive time headaway list, starting index is indicated by the parameter: "start". 
    std::vector<double> PlatoonManager::getTimeHeadwayFromIndex(std::vector<double> timeHeadways, int start) const {
        std::vector<double> result(timeHeadways.begin() + start-1, timeHeadways.end());
        return result;
    }
    
    // Determine if the gap (m) between host and predecessor is big enough, with regards to minGap_ (m) and maxGap_ (m).
    bool PlatoonManager::insufficientGapWithPredecessor(double distanceToPredVehicle) {
        
        // For normal operation, gap > minGap is necessary. 
        bool frontGapIsTooSmall = distanceToPredVehicle < config_.minCutinGap; 
        
        // Host vehicle was following predecessor vehicle. --> The predecessor vehicle was violating gap threshold.
        bool previousLeaderIsPredecessor = previousFunctionalDynamicLeaderID_ == host_platoon_[host_platoon_.size() - 1].staticId; 
        
        // Gap greater than maxGap_ is necessary for host to stop choosing predecessor as dynamic leader. 
        bool frontGapIsNotLargeEnough = distanceToPredVehicle < config_.maxCutinGap && previousLeaderIsPredecessor;

        return (frontGapIsTooSmall || frontGapIsNotLargeEnough);
    }

    // Calculate the time headway (s) behind each vehicle of the platoon. If no one behind or following car stoped, return infinity.
    std::vector<double> PlatoonManager::calculateTimeHeadway(std::vector<double> downtrackDistance, std::vector<double> speed) const{
        std::vector<double> timeHeadways(downtrackDistance.size() - 1);
        // Due to downtrack descending order, the platoon member with smaller index has larger downtrack, hence closer to the front of the platoon.
        for (size_t i = 0; i < timeHeadways.size(); i++){
            // Calculate time headaway between the vehicle behind. 
            if (speed[i+1] >= config_.ss_theta) // speed is in m/s
            {
                timeHeadways[i] = (downtrackDistance[i] - downtrackDistance[i+1]) / speed[i+1]; // downtrack is in m, speed is in m/s.
            }
            // If no one behind or following car stoped, return infinity. 
            else
            {
                timeHeadways[i] = std::numeric_limits<double>::infinity();
            }
        }
        return timeHeadways; // time is in s.
    }

    // Determine the dynamic leader ID based on gap threshold violation's index.
    int PlatoonManager::determineDynamicLeaderBasedOnViolation(std::vector<double> timeHeadways){
        
        /**
         *  Note: For both condition, the host will always choose to follow the vechile that has a relatively larger gap in front.
         *                                
         *   
         *  max-vilation (follow veh2):   [*veh3*] ---------- [*veh2*] ------------------------------- [*veh1*] ---------- [*veh0*]
         *                                                       ^
         *                                            gap2                    gap1(max violation)                  gap0
         *  
         *  min-vilation (follow veh1):   [*veh3*] -----------[*veh2*]---[*veh1*] ---------- [*veh0*]
         *                                                                  ^
         *                                            gap2            gap1              gap0
         *                                                       (min violation)                           
         */
        // Find the closest violations. 
        int closestLowerBoundaryViolation = findLowerBoundaryViolationClosestToTheHostVehicle(timeHeadways);
        int closestMaximumSpacingViolation = findMaximumSpacingViolationClosestToTheHostVehicle(timeHeadways);
        
        // Compare the violation locations, always following the closer violation vehicle (larger index) first, then the furthur ones.
        if(closestLowerBoundaryViolation > closestMaximumSpacingViolation) {
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_strategic_ihp"), "APF found violation on closestLowerBoundaryViolation at " << closestLowerBoundaryViolation);
            return closestLowerBoundaryViolation; // Min violation, following the vehicle that is in font of the violating gap.
        } 
        else if(closestLowerBoundaryViolation < closestMaximumSpacingViolation){
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_strategic_ihp"), "APF found violation on closestMaximumSpacingViolation at " << closestMaximumSpacingViolation);
            return closestMaximumSpacingViolation + 1; // Max violation, follow the vehicle that is behinf the violating gap.
        }
        else{
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_strategic_ihp"), "APF found no violations on both closestLowerBoundaryViolation and closestMaximumSpacingViolation");
            return 0;
        }
    }

    // Find the lower boundary violation vehicle that closest to the host vehicle. If no violation found, return -1.
    int PlatoonManager::findLowerBoundaryViolationClosestToTheHostVehicle(std::vector<double> timeHeadways) const{
        // Due to descending downtrack order, the search starts from the platoon rear, which corresponds to last in list.
        for(int i = timeHeadways.size()-1; i >= 0; i--) {
            if(timeHeadways[i] < config_.minAllowableHeadaway)  // in s
            {
                return i;
            }
        }
        return -1;
    }
    
    // Find the maximum spacing violation vehicle that closest to the host vehicle. If no violation found, return -1.
    int PlatoonManager::findMaximumSpacingViolationClosestToTheHostVehicle(std::vector<double> timeHeadways) const {
        // UCLA: Add maxAllowableHeadaway adjuster to increase the threshold during gap creating period.
        double maxAllowableHeadaway_adjusted = config_.maxAllowableHeadaway;
        if (isCreateGap) {
            // adjust maximum allowed headway to allow for a bigger gap 
            maxAllowableHeadaway_adjusted = maxAllowableHeadaway_adjusted*(1 + config_.createGapAdjuster);
            } 
        
        //  Due to descending downtrack order, the search starts from the platoon rear, which corresponds to last in list.
        for(int i = timeHeadways.size()-1; i >= 0 ; i--) {
            if(timeHeadways[i] > maxAllowableHeadaway_adjusted) // in s
            {
                return i;
            }
        }
        return -1;
    }

    // Change the local platoon manager from follower operation state to leader operation state for single vehicle status change. 
    // This could happen because host is 2nd in line and leader is departing, or because APF algorithm decided host's gap is too
    // large and we need to separate from front part of platoon.
    void PlatoonManager::changeFromFollowerToLeader() {
        
        // Get current host info - assumes departing leader or front of platoon hasn't already been removed from the vector
        PlatoonMember hostInfo = host_platoon_[hostPosInPlatoon_];
        
        // Clear the front part of the platoon info, since we are splitting off from it; leaves host as element 0
        if (hostPosInPlatoon_ > 0)
        {
            host_platoon_.erase(host_platoon_.begin(), host_platoon_.begin() + hostPosInPlatoon_);
        }else
        {
            RCLCPP_WARN(rclcpp::get_logger("platoon_strategic_ihp"), "### Host becoming leader, but is already at index 0 in host_platoon_ vector!  Vector unchanged.");
        }

        hostPosInPlatoon_ = 0;
        isFollower = false;
        currentPlatoonID = boost::uuids::to_string(boost::uuids::random_generator()());
        previousFunctionalDynamicLeaderID_ = "";
        previousFunctionalDynamicLeaderIndex_ = -1;
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_strategic_ihp"), "The platoon manager is changed from follower state to leader state. New platoon ID = " << currentPlatoonID);
    }

    // Change the local platoon manager from leader operation state to follower operation state for single vehicle status change.
    // This could happen because another vehicle did a front join and host is now 2nd in line, or because host was a solo vehicle
    // that just completed joining a platoon at any position.
    // Note: The platoon list will be firstly reset to only include the new leader and the host, then allowed to gradually repopulate via
    // updateMembers() as new MobilityOperation messages come in.
    void PlatoonManager::changeFromLeaderToFollower(std::string newPlatoonId, std::string newLeaderId) {
        
        // Save the current host info
        PlatoonMember hostInfo = host_platoon_[hostPosInPlatoon_];
        
        // Clear contents of the platoon vector and rebuild it with the two known members at this time, leader & host.
        // Remaining leader info and info about any other members will get populated as messages come in.
        PlatoonMember newLeader = PlatoonMember();
        newLeader.staticId = newLeaderId;
        host_platoon_.clear();
        host_platoon_.push_back(newLeader); //can get location info updated later with a STATUS or INFO message
        host_platoon_.push_back(hostInfo);
 
        hostPosInPlatoon_ = 1; //since host was previously leader it is now guaranteed to be 2nd in the line (index 1)
        isFollower = true;
        currentPlatoonID = newPlatoonId;

        // Clear the record of neighbor platoon, since we likely just joined it
        resetNeighborPlatoon();

        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_strategic_ihp"), "The platoon manager is changed from leader state to follower state. Platoon vector re-initialized. Plan ID = " << newPlatoonId);
    }

    // Return the number of vehicles in the front of the host vehicle. If host is leader or a single vehicle, return 0.
    int PlatoonManager::getNumberOfVehicleInFront() {
        return hostPosInPlatoon_;
    }

    // Return the distance (m) to the predecessor vehicle.
    double PlatoonManager::getDistanceToPredVehicle() {
        return gapWithPred_;
    }

    // Return the current host vehicle speed in m/s.
    double PlatoonManager::getCurrentSpeed() const {
        return host_platoon_[hostPosInPlatoon_].vehicleSpeed;
    }

    // Return the current command speed of host vehicle in m/s.
    double PlatoonManager::getCommandSpeed() const
    {
        return host_platoon_[hostPosInPlatoon_].commandSpeed;
    }

    // Return the current downtrack distance in m.
    double PlatoonManager::getCurrentDowntrackDistance() const
    {
        return host_platoon_[hostPosInPlatoon_].vehiclePosition;
    }

    // Return the current crosstrack distance, in m.
    double PlatoonManager::getCurrentCrosstrackDistance() const
    {
        return host_platoon_[hostPosInPlatoon_].vehicleCrossTrack;
    }

    // UCLA: return the host vehicle static ID.
    std::string PlatoonManager::getHostStaticID() const
    {
        return HostMobilityId;
    }

    // Return the physical length from platoon front vehicle (front bumper) to platoon rear vehicle (rear bumper) in m.
    double PlatoonManager::getCurrentPlatoonLength() {
        //this works even if platoon size is 1 (can't be 0)
        return host_platoon_[0].vehiclePosition - host_platoon_[host_platoon_.size() - 1].vehiclePosition + config_.vehicleLength; 
    }

    // ---------------------- UCLA: IHP platoon trajectory regulation --------------------------- //
    // Calculate the time headway summation of the vehicle in front of the host
    double PlatoonManager::getPredecessorTimeHeadwaySum()
    {
        // 1. read dtd vector 
        // dtd vector 
        std::vector<double> downtrackDistance(host_platoon_.size());
        for (size_t i = 0; i < host_platoon_.size(); i++)
        {
            downtrackDistance[i] = host_platoon_[i].vehiclePosition;
        }
        // speed vector
        std::vector<double> speed(host_platoon_.size());
        for (size_t i = 0; i < host_platoon_.size(); i++)
        {
            speed[i] = host_platoon_[i].vehicleSpeed;
        }

        // 2. find the summation of "veh_len/veh_speed" for all predecessors
        double tmp_time_hdw = 0.0;
        double cur_dtd;
        for (size_t index = 0; index < downtrackDistance.size(); index++)
        {
            cur_dtd = downtrackDistance[index];
            if (cur_dtd > getCurrentDowntrackDistance())
            {
                // greater dtd ==> in front of host veh 
                tmp_time_hdw += config_.vehicleLength / (speed[index] + 0.00001);
            }
        }

        // Return the calcualted value
        return tmp_time_hdw;
    }

    // Return the predecessor speed 
    double PlatoonManager::getPredecessorSpeed()
    {
        return host_platoon_[hostPosInPlatoon_].vehicleSpeed; // m/s 
    }

    // Return the predecessor location
    double PlatoonManager::getPredecessorPosition()
    {
        // Read host index. 
        int host_platoon_index = getNumberOfVehicleInFront();

        // Return speed
        return host_platoon_[host_platoon_index].vehiclePosition; // m
    }

    // Trajectory based platoon trajectory regulation.
    double PlatoonManager::getIHPDesPosFollower(double time_step)
    {
        /**
         * Calculate desired position based on previous vehicle's trajectory for followers.
         * 
         * TODO: The platoon trajectory regulation is derived with the assumption that all vehicle 
         *       have identical length (i.e., 5m). Future development is needed to include variable 
         *       vehicle length into consideration.
         */

        // 1. read dtd vector 
        // dtd vector 
        std::vector<double> downtrackDistance(host_platoon_.size());
        for (size_t i = 0; i < host_platoon_.size(); i++)
        {
            downtrackDistance[i] = host_platoon_[i].vehiclePosition;
        }
        // speed vector
        std::vector<double> speed(host_platoon_.size());
        for (size_t i = 0; i < host_platoon_.size(); i++)
        {
            speed[i] = host_platoon_[i].vehicleSpeed;
        }

        // 2. find the summation of "veh_len/veh_speed" for all predecessors
        double tmp_time_hdw = 0.0;
        double cur_dtd;
        for (size_t index = 0; index < downtrackDistance.size(); index++)
        {
            cur_dtd = downtrackDistance[index];
            if (cur_dtd > getCurrentDowntrackDistance())
            {
                // greater dtd ==> in front of host veh 
                tmp_time_hdw += config_.vehicleLength / (speed[index] + 0.00001);
            }
        }

        // 3. read host veh and front veh info 
        // Predecessor vehicle data.
        double pred_spd = speed[getNumberOfVehicleInFront()-1]; // m/s 
        double pred_pos = downtrackDistance[getNumberOfVehicleInFront()-1]; // m
        
        // host data. 
        double ego_spd = getCurrentSpeed(); // m/s
        double ego_pos = getCurrentDowntrackDistance(); // m
        
        // platoon position index
        int pos_idx = getNumberOfVehicleInFront();

        double desirePlatoonGap = config_.intra_tau; //s
        
        // IHP desired position calculation methods
        double pos_g; // desired downtrack position calculated with time-gap, in m.
        double pos_h; // desired downtrack position calculated with distance headaway, in m.

        // 4. IHP gap regualtion 
        
        // intermediate variables 
        double timeGapAndStepRatio = desirePlatoonGap/time_step;      // The ratio between desired platoon time gap and the current time_step.
        double totalTimeGap = desirePlatoonGap*pos_idx;               // The overall time gap from host vehicle to the platoon leader, in s.

        // calcilate pos_gap and pos_headway
        if ((pred_spd <= ego_spd) && (ego_spd <= config_.ss_theta))
        {
            // ---> 4.1 pos_g 
            pos_g = (pred_pos - config_.vehicleLength - config_.standstill + ego_pos*timeGapAndStepRatio) / (1 + timeGapAndStepRatio);
            // ---> 4.2 pos_h
            double pos_h_nom = (pred_pos - config_.standstill + ego_pos*(totalTimeGap + tmp_time_hdw)/time_step);
            double pos_h_denom = (1 + ((totalTimeGap + tmp_time_hdw)/time_step));
            pos_h = pos_h_nom/pos_h_denom;

        }
        else
        {   
            // ---> 4.1 pos_g 
            pos_g = (pred_pos - config_.vehicleLength + ego_pos*(timeGapAndStepRatio)) / (1 + timeGapAndStepRatio);
            // ---> 4.2 pos_h
            double pos_h_nom = (pred_pos + ego_pos*(totalTimeGap + tmp_time_hdw)/time_step);
            double pos_h_denom = (1 + ((totalTimeGap + tmp_time_hdw)/time_step));
            pos_h = pos_h_nom/pos_h_denom;
        }

        // ---> 4.3 desire speed and desire location 
        double pos_des = config_.gap_weight*pos_g + (1.0 - config_.gap_weight)*pos_h;
        // double des_spd = (pos_des - ego_pos) / time_step;

        // ---> 4.4 return IHP calculated desired location
        return pos_des;
    }

    // UCLA: find the index of the closest vehicle in the target platoon that is in front of the host vehicle (cut-in joiner).
    // Note: The joiner will cut-in at the back of this vehicle, which make this index points to the vehicle that is leading the cut-in gap.
    int PlatoonManager::getClosestIndex(double joinerDtD)
    {   
        /*
            A naive way to find the closest member that is in front of the host that point to the gap to join the platoon.
            Note: The potential gap leading vehicle should be in front of the joiner (i.e., gap leading vehicle's dtd > joiner's dtd).
                  If the joiner is already in front of the platoon leader, this function will return -1 (i.e., cut-in front).
        */

        double min_diff = 99999.0;
        int cut_in_index = -1; //-2 is meaningless default

        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_strategic_ihp"), "neighbor_platoon_.size(): " << neighbor_platoon_.size());

        // Loop through all target platoon members  
        for(size_t i = 0; i < neighbor_platoon_.size(); i++) 
        {
            double current_member_dtd = neighbor_platoon_[i].vehiclePosition; 
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_strategic_ihp"), "current_member_dtd: "<< current_member_dtd);
            double curent_dtd_diff = current_member_dtd - joinerDtD;
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_strategic_ihp"), "curent_dtd_diff: "<< curent_dtd_diff);
            // update min index
            if (curent_dtd_diff > 0 && curent_dtd_diff < min_diff)
            {
                min_diff = current_member_dtd;
                cut_in_index = i;
            }
        }
        
        return cut_in_index;
    }

    // UCLA: find the current cut-in join gap size in downtrack distance (m). The origin of the vehicle when calculating DtD is locate at the rear axle. 
    double PlatoonManager::getCutInGap(const int gap_leading_index, const double joinerDtD)
    {
        /*
            Locate the target cut-in join gap size based on the index.   
        */

        // Initiate variables 
        double gap_size = -0.999;
        size_t index = 0;
        if (gap_leading_index >= 0)
        {
            index = static_cast<size_t>(gap_leading_index);
        }

        // cut-in from front 
        if (gap_leading_index == -1)
        {
            double gap_rear_dtd = neighbor_platoon_[0].vehiclePosition;
            gap_size = joinerDtD - gap_rear_dtd - config_.vehicleLength;
        }
        // cut-in from behind 
        else if (index == neighbor_platoon_.size() - 1)
        {    
            double gap_leading_dtd = neighbor_platoon_[index].vehiclePosition;
            gap_size = gap_leading_dtd - joinerDtD - config_.vehicleLength;;
        }
        // cut-in in the middle
        else
        {
            double gap_leading_dtd = neighbor_platoon_[index].vehiclePosition;
            double gap_rear_dtd = neighbor_platoon_[index + 1].vehiclePosition;
            gap_size = gap_leading_dtd - gap_rear_dtd - config_.vehicleLength;
        }

        // return gap_size value
        return gap_size;
    }
}