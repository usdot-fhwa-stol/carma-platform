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

#include "platoon_manager_ihp.h"
#include "platoon_config_ihp.h"
#include <boost/algorithm/string.hpp>
#include <ros/ros.h>
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

    PlatoonManager::PlatoonManager()
    {
        ROS_DEBUG_STREAM("Top of PlatoonManager ctor.");
    }


    // Update the location of the host in the vector of platoon members
    void PlatoonManager::updateHostPose(const double downtrack, const double crosstrack)
    {
        ROS_DEBUG_STREAM("Host (index " << hostPosInPlatoon_ << "): downtrack = " << downtrack << ", crosstrack = " << crosstrack);
        platoon[hostPosInPlatoon_].vehiclePosition = downtrack;
        platoon[hostPosInPlatoon_].vehicleCrossTrack = crosstrack;
    }

    // Update the speed info of the host in the vector of platoon members
    void PlatoonManager::updateHostSpeeds(const double cmdSpeed, const double actualSpeed)
    {
        platoon[hostPosInPlatoon_].commandSpeed = cmdSpeed;
        platoon[hostPosInPlatoon_].vehicleSpeed = actualSpeed;
    }

    // Update/add one member's information from STATUS messages, update platoon ID if needed. Ignore if message is from other platoons. 
    void PlatoonManager::memberUpdates(const std::string& senderId, const std::string& platoonId, const std::string& params, const double& DtD, const double& CtD){

        // parase params, read member data
        std::vector<std::string> inputsParams;
        boost::algorithm::split(inputsParams, params, boost::is_any_of(","));
        // read command speed, m/s
        std::vector<std::string> cmd_parsed;
        boost::algorithm::split(cmd_parsed, inputsParams[0], boost::is_any_of(":"));
        double cmdSpeed = std::stod(cmd_parsed[1]);
        ROS_DEBUG_STREAM("Command Speed: " << cmdSpeed);
        // get DtD directly instead of parsing message, m
        double dtDistance = DtD;
        // get CtD directly 
        double ctDistance = CtD;
        ROS_DEBUG_STREAM("Downtrack Distance ecef: " << dtDistance);
        ROS_DEBUG_STREAM("CrossTrack Distance ecef: " <<ctDistance);
        // read current speed, m/s
        std::vector<std::string> cur_parsed;
        boost::algorithm::split(cur_parsed, inputsParams[1], boost::is_any_of(":"));
        double curSpeed = std::stod(cur_parsed[1]);
        ROS_DEBUG_STREAM("Current Speed Speed: " << curSpeed);

        // If we are currently in a follower state:
        // 1. We will update platoon ID based on leader's STATUS
        // 2. We will update platoon members info based on platoon ID for all members
        if (isFollower) 
        {
            // read message status        
            bool isFromLeader = platoon[0].staticId == senderId;
            bool needPlatoonIdChange = isFromLeader && (currentPlatoonID != platoonId);

            if(needPlatoonIdChange)
            {
                // TODO: better to have leader explicitly inform us that it is merging into another platoon, rather than require us to deduce it here.
                //       This condition may also result from missing some message traffic, new leader in this platoon, or other confusion/incorrect code.
                //       Consider using a new STATUS msg param that tells members of new platoon ID and new leader ID when a change is made.
                ROS_DEBUG_STREAM("It seems that the current leader is joining another platoon.");
                ROS_DEBUG_STREAM("So the platoon ID is changed from " << currentPlatoonID << " to " << platoonId);
                currentPlatoonID = platoonId;
                updatesOrAddMemberInfo(senderId, cmdSpeed, dtDistance, ctDistance, curSpeed);
            } 
            else if (currentPlatoonID == platoonId)
            {
                ROS_DEBUG_STREAM("This STATUS messages is from our platoon. Updating the info...");
                updatesOrAddMemberInfo(senderId, cmdSpeed, dtDistance, ctDistance, curSpeed);
                ROS_DEBUG_STREAM("The first vehicle in our list is now " << platoon[0].staticId);
            } 
            else //sender is in a different platoon
            {
                ROS_DEBUG_STREAM("This STATUS message is not from a vehicle we care about. Ignore this message with id: " << senderId);
            }
        }
        else //host is leader
        {
            // If we are currently in any leader state, we only update platoon member based on platoon ID
            ROS_DEBUG_STREAM("Host is leader: currentPlatoonID = " << currentPlatoonID << ", incoming platoonId = " << platoonId);
            if (currentPlatoonID == platoonId)
            {
                ROS_DEBUG_STREAM("This STATUS messages is from our platoon. Updating the info...");
                updatesOrAddMemberInfo(senderId, cmdSpeed, dtDistance, ctDistance, curSpeed);
            }
            else
            {
                ROS_DEBUG_STREAM("Platoon IDs not matched");
                ROS_DEBUG_STREAM("currentPlatoonID: " << currentPlatoonID);
                ROS_DEBUG_STREAM("incoming platoonId: " << platoonId);
            }
        }
        
        // update host vehicle information each time new member is updated. Now platoon contains host vehicle.
        std::string hostStaticId = getHostStaticID();
        double hostcmdSpeed = getCommandSpeed();
        double hostDtD = getCurrentDowntrackDistance();
        double hostCtD = getCurrentCrosstrackDistance();
        double hostCurSpeed = getCurrentSpeed();
        updatesOrAddMemberInfo(hostStaticId, hostcmdSpeed, hostDtD, hostCtD, hostCurSpeed);
    }
    
    // Check a new vehicle's existence; add its info to the platoon if not in list, update info if already existed. 
    void PlatoonManager::updatesOrAddMemberInfo(std::string senderId, double cmdSpeed, double dtDistance, double ctDistance, double curSpeed)
    {

        bool isExisted = false;

        // update/add this info into the list
        for (int i = 0;  i < platoon.size();  ++i){
            if(platoon[i].staticId == senderId) {
                platoon[i].commandSpeed = cmdSpeed;         // m/s
                platoon[i].vehiclePosition = dtDistance;    // m 
                platoon[i].vehicleCrossTrack = ctDistance;  // m
                platoon[i].vehicleSpeed = curSpeed;         // m/s
                platoon[i].timestamp = ros::Time::now().toNSec()/1000000;
                ROS_DEBUG_STREAM("Receive and update platooning info on member " << i << ", ID:" << platoon[i].staticId);
                ROS_DEBUG_STREAM("    CommandSpeed       = " << platoon[i].commandSpeed);
                ROS_DEBUG_STREAM("    Actual Speed       = " << platoon[i].vehicleSpeed);
                ROS_DEBUG_STREAM("    Downtrack Location = " << platoon[i].vehiclePosition);
                ROS_DEBUG_STREAM("    Crosstrack dist    = " << platoon[i].vehicleCrossTrack);

                if (senderId == HostMobilityId)
                {
                    hostPosInPlatoon_ = i;
                    ROS_DEBUG_STREAM("    This is the HOST vehicle");
                }
                isExisted = true;
                break;
            }
        }

        // if not already exist, add to platoon list.
        if(!isExisted) {
            long cur_t = ros::Time::now().toNSec()/1000000; // time in millisecond

            PlatoonMember newMember = PlatoonMember(senderId, cmdSpeed, curSpeed, dtDistance, ctDistance, cur_t);
            platoon.push_back(newMember);
            // sort the platoon member based on dowtrack distance (m) in an descending order.
            std::sort(std::begin(platoon), std::end(platoon), [](const PlatoonMember &a, const PlatoonMember &b){return a.vehiclePosition > b.vehiclePosition;});

            ROS_DEBUG_STREAM("Add a new vehicle into our platoon list " << newMember.staticId << " platoon.size now = " << platoon.size());
            ROS_DEBUG_STREAM("    Platoon order is now:");
            for (int i = 0;  i < platoon.size();  ++i)
            {
                std::string hostFlag = " ";
                if (i == hostPosInPlatoon_)
                {
                    hostFlag = "Host";
                }
                ROS_DEBUG_STREAM("    " << platoon[i].staticId << " " << hostFlag);
            }
        }
    }

    // Get the platoon size.
    int PlatoonManager::getTotalPlatooningSize() {
        ROS_DEBUG_STREAM("platoonSize: " << platoon.size());
        return platoon.size();
    }
        
    // Find the downtrack distance of the last vehicle of the platoon, in m.    
    double PlatoonManager::getPlatoonRearDowntrackDistance(){
        // due to downtrack descending order, the 1ast vehicle in list is the platoon rear vehicle.
        // Even if host is solo, platoon size is 1 so this works.
        return platoon[platoon.size()-1].vehiclePosition;
    }

    // Find the downtrack distance of the first vehicle of the platoon, in m.
    double PlatoonManager::getPlatoonFrontDowntrackDistance(){
        // due to downtrack descending order, the firest vehicle in list is the platoon front vehicle. 
        return platoon[0].vehiclePosition;
    }

    // Return the dynamic leader (i.e., the vehicle to follow) of the host vehicle.
    PlatoonMember PlatoonManager::getDynamicLeader(){
        PlatoonMember dynamicLeader;
        ROS_DEBUG_STREAM("platoon size: " << platoon.size());
        if(isFollower) 
        {
            ROS_DEBUG_STREAM("Leader initially set as first vehicle in platoon");
            // return the first vehicle in the platoon as default if no valid algorithm applied
            // due to downtrack descending order, the platoon front veihcle is the first in list. 
            dynamicLeader = platoon[0];
            if (algorithmType_ == "APF_ALGORITHM"){
                int newLeaderIndex = allPredecessorFollowing();
                if(newLeaderIndex < platoon.size() && newLeaderIndex >= 0) { //this must always be true!
                    dynamicLeader = platoon[newLeaderIndex];
                    ROS_DEBUG_STREAM("APF output: " << dynamicLeader.staticId);
                    previousFunctionalDynamicLeaderIndex_ = newLeaderIndex;
                    previousFunctionalDynamicLeaderID_ = dynamicLeader.staticId;
                }
                else //something is terribly wrong in the logic!
                {
                    ROS_WARN("newLeaderIndex = ", newLeaderIndex, " is invalid coming from allPredecessorFollowing!");
                    /**
                     * it might happened when the subject vehicle gets far away from the preceding vehicle, 
                     * in which case the host vehicle will follow the one in front.
                     */
                    dynamicLeader = platoon[getNumberOfVehicleInFront() - 1];
                    // update index and ID 
                    previousFunctionalDynamicLeaderIndex_ = getNumberOfVehicleInFront()-1;
                    previousFunctionalDynamicLeaderID_ = dynamicLeader.staticId;
                    ROS_DEBUG_STREAM("Based on the output of APF algorithm we start to follow our predecessor.");
                }
            }
        }
        return dynamicLeader;

    }

    // The implementation of all predecessor following algorithm. Determine the dynamic leader for the host vheicle to follow.
    int PlatoonManager::allPredecessorFollowing(){
        ///***** Case Zero *****///
        // If the host vheicle is the second vehicle in this platoon,we will always follow the platoon leader in front of host vehicle
        if(platoon.size() == 2) {
            ROS_DEBUG("As the second vehicle in the platoon, it will always follow the leader. Case Zero");
            return 0;
        }
        ///***** Case One *****///
        // If there weren't a leader in the previous time step, follow the first vehicle (i.e., the platoon leader) as default.
        if(previousFunctionalDynamicLeaderID_ == "") {
            ROS_DEBUG("APF algorithm did not found a dynamic leader in previous time step. Case one");
            return 0;
        }

        //***** Formulate speed and downtrack vector *****//
        // Update host vehicle info when update member info, so platoon list include host vehicle, direct use platoon size for downtrack/speed vector.
        // Record downtrack distance (m) of each member
        std::vector<double> downtrackDistance(platoon.size());
        for(int i = 0; i < platoon.size(); i++) {
            downtrackDistance[i] = platoon[i].vehiclePosition; // m
        }
        // Record speed (m/s) of each member
        std::vector<double> speed(platoon.size());
        for(int i = 0; i < platoon.size(); i++) {
            speed[i] = platoon[i].vehicleSpeed; // m/s
        }
        

        ///***** Case Two *****///
        // If the distance headway between the subject vehicle and its predecessor is an issue
        // according to the "min_gap" and "max_gap" thresholds, then it should follow its predecessor
        // The following line will not throw exception because the length of downtrack array is larger than two in this case
        double distHeadwayWithPredecessor = downtrackDistance[downtrackDistance.size() - 2] - downtrackDistance[downtrackDistance.size() - 1];
        gapWithPred_ = distHeadwayWithPredecessor;
        if(insufficientGapWithPredecessor(distHeadwayWithPredecessor)) {
            ROS_DEBUG("APF algorithm decides there is an issue with the gap with preceding vehicle: " , distHeadwayWithPredecessor , " m. Case Two");
            return platoon.size() - 1;
        }
        else{
            // implementation of the main part of APF algorithm
            // calculate the time headway between every consecutive pair of vehicles
            std::vector<double> timeHeadways = calculateTimeHeadway(downtrackDistance, speed);
            ROS_DEBUG("APF calculate time headways: " );
            for (const auto& value : timeHeadways)
            {
                ROS_DEBUG("APF time headways: " , value);
            }
            ROS_DEBUG("APF found the previous dynamic leader is " , previousFunctionalDynamicLeaderID_);
            // if the previous dynamic leader is the first vehicle in the platoon
            
            if(previousFunctionalDynamicLeaderIndex_ == 0) {

                ///***** Case Three *****///
                // If there is a violation, the return value is the desired dynamic leader index
                ROS_DEBUG("APF use violations on lower boundary or maximum spacing to choose dynamic leader. Case Three.");
                return determineDynamicLeaderBasedOnViolation(timeHeadways);
            }
            else{
                // if the previous dynamic leader is not the first vehicle
                // get the time headway between every consecutive pair of vehicles from index Of Previous dynamic Leader
                std::vector<double> partialTimeHeadways = getTimeHeadwayFromIndex(timeHeadways, previousFunctionalDynamicLeaderIndex_);
                ROS_DEBUG("APF partial time headways array:: " );
                for (const auto& value : partialTimeHeadways)
                {
                    ROS_DEBUG("APF partial time headways: " , value);
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
                        ROS_DEBUG("APF found two conditions for assigning local dynamic leadership further downstream are satisfied. Case Four");
                        return determineDynamicLeaderBasedOnViolation(timeHeadways);
                    } else {
                        
                        ///***** Case Five *****///
                        // We may not switch dynamic leadership to another vehicle further downstream because some criteria are not satisfied
                        ROS_DEBUG("APF found two conditions for assigning local dynamic leadership further downstream are not satisfied. Case Five.");
                        ROS_DEBUG("condition1: " , condition1 , " & condition2: " , condition2);
                        return previousFunctionalDynamicLeaderIndex_;
                    }
                } else if(closestLowerBoundaryViolation != -1 && closestMaximumSpacingViolation == -1) {
                    // The rest four cases have roughly the same logic: locate the closest violation and assign dynamic leadership accordingly
                    
                    ///***** Case Six *****///
                    ROS_DEBUG("APF found closestLowerBoundaryViolation on partial time headways. Case Six.");
                    return previousFunctionalDynamicLeaderIndex_ - 1 + closestLowerBoundaryViolation;

                } else if(closestLowerBoundaryViolation == -1 && closestMaximumSpacingViolation != -1) {
                    
                    ///***** Case Seven *****///
                    ROS_DEBUG("APF found closestMaximumSpacingViolation on partial time headways. Case Seven.");
                    return previousFunctionalDynamicLeaderIndex_ + closestMaximumSpacingViolation;
                } else{
                    ROS_DEBUG("APF found closestMaximumSpacingViolation and closestLowerBoundaryViolation on partial time headways.");
                    if(closestLowerBoundaryViolation > closestMaximumSpacingViolation) {
                        
                        ///***** Case Eight *****///
                        ROS_DEBUG("closest LowerBoundaryViolation is higher than closestMaximumSpacingViolation on partial time headways. Case Eight.");
                        return previousFunctionalDynamicLeaderIndex_ - 1 + closestLowerBoundaryViolation;
                    } else if(closestLowerBoundaryViolation < closestMaximumSpacingViolation) {
                        
                        ///***** Case Nine *****///
                        ROS_DEBUG("closestMaximumSpacingViolation is higher than closestLowerBoundaryViolation on partial time headways. Case Nine.");
                        return previousFunctionalDynamicLeaderIndex_ + closestMaximumSpacingViolation;
                    } else {
                        ROS_DEBUG("APF dynamic Leader selection parameter is wrong!");
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
        bool frontGapIsTooSmall = distanceToPredVehicle < config_.minGap; 
        
        // Host vehicle was following predecessor vehicle. --> The predecessor vehicel was violating gap threshold.
        bool previousLeaderIsPredecessor = previousFunctionalDynamicLeaderID_ == platoon[platoon.size() - 1].staticId; 
        
        // Gap greater than maxGap_ is necessary for host to stop choosing predecessor as dynamic leader. 
        bool frontGapIsNotLargeEnough = (distanceToPredVehicle < config_.maxGap && previousLeaderIsPredecessor);

        return (frontGapIsTooSmall || frontGapIsNotLargeEnough);
    }

    // Calculate the time headway (s) behind each vehicle of the platoon. If no one behind or following car stoped, return infinity.
    std::vector<double> PlatoonManager::calculateTimeHeadway(std::vector<double> downtrackDistance, std::vector<double> speed) const{
        std::vector<double> timeHeadways(downtrackDistance.size() - 1);
        // Due to downtrack descending order, the platoon member with smaller index has larger downtrack, hence closer to the front of the platoon.
        for (int i=0; i<timeHeadways.size(); i++){
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
            ROS_DEBUG("APF found violation on closestLowerBoundaryViolation at " , closestLowerBoundaryViolation);
            return closestLowerBoundaryViolation; // Min violation, following the vehicle that is in font of the violating gap.
        } 
        else if(closestLowerBoundaryViolation < closestMaximumSpacingViolation){
            ROS_DEBUG("APF found violation on closestMaximumSpacingViolation at " , closestMaximumSpacingViolation);
            return closestMaximumSpacingViolation + 1; // Max violation, follow the vehicle that is behinf the violating gap.
        }
        else{
            ROS_DEBUG("APF found no violations on both closestLowerBoundaryViolation and closestMaximumSpacingViolation");
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
        //  Due to descending downtrack order, the search starts from the platoon rear, which corresponds to last in list.
        for(int i = timeHeadways.size()-1; i >= 0 ; i--) {
            if(timeHeadways[i] > config_.maxAllowableHeadaway) // in s
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
        PlatoonMember hostInfo = platoon[hostPosInPlatoon_];
        
        // Clear the front part of the platoon info, since we are splitting off from it; leaves host as element 0
        if (hostPosInPlatoon_ > 0)
        {
            platoon.erase(platoon.begin(), platoon.begin() + hostPosInPlatoon_);
        }else
        {
            ROS_WARN("### Host becoming leader, but is already at index 0 in platoon vector!  Vector unchanged.");
        }

        hostPosInPlatoon_ = 0;
        isFollower = false;
        currentPlatoonID = boost::uuids::to_string(boost::uuids::random_generator()());
        previousFunctionalDynamicLeaderID_ = "";
        previousFunctionalDynamicLeaderIndex_ = -1;
        ROS_DEBUG_STREAM("The platoon manager is changed from follower state to leader state. New platoon ID = " << currentPlatoonID);
    }

    // Change the local platoon manager from leader operation state to follower operation state for single vehicle status change.
    // This could happen because another vehicle did a front join and host is now 2nd in line, or because host was a solo vehicle
    // that just completed joining a platoon at any position.
    // Note: The platoon list will be firstly reset to only include the new leader and the host, then allowed to gradually repopulate via
    // updateMembers() as new MobilityOperation messages come in.
    void PlatoonManager::changeFromLeaderToFollower(std::string newPlatoonId, std::string newLeaderId) {
        
        // Save the current host info
        PlatoonMember hostInfo = platoon[hostPosInPlatoon_];
        
        // Clear contents of the platoon vector and rebuild it with the two known members at this time, leader & host.
        // Remaining leader info and info about any other members will get populated as messages come in.
        PlatoonMember newLeader = PlatoonMember();
        newLeader.staticId = newLeaderId;
        platoon.clear();
        platoon.push_back(newLeader); //can get location info updated later with a STATUS or INFO message
        platoon.push_back(hostInfo);
 
        hostPosInPlatoon_ = 1;
        isFollower = true;
        currentPlatoonID = newPlatoonId;
        ROS_DEBUG_STREAM("The platoon manager is changed from leader state to follower state. Platoon vector re-initialized. Plan ID = " << newPlatoonId);
    }

    // Return the number of vehicles in the front of the host vehicle. If host is leader or a single vehicle, return 0.
    int PlatoonManager::getNumberOfVehicleInFront() {
        
        int num = platoon.size() - 1;
        for (int i = 0;  i < platoon.size();  ++i)
        {
            if (platoon[i].vehiclePosition <= getCurrentDowntrackDistance() + 1.0) //allow for some uncertainty to count host also
            {
                num = i;
                break;
            }
        }
        return num;
    }

    // Return the distance (m) to the predecessor vehicle.
    double PlatoonManager::getDistanceToPredVehicle() {
        return gapWithPred_;
    }

    // Return the current host vehicle speed in m/s.
    double PlatoonManager::getCurrentSpeed() const {
        return platoon[hostPosInPlatoon_].vehicleSpeed;
    }

    // Return the current command speed of host vehicle in m/s.
    double PlatoonManager::getCommandSpeed() const
    {
        return platoon[hostPosInPlatoon_].commandSpeed;
    }

    // Return the current downtrack distance in m.
    double PlatoonManager::getCurrentDowntrackDistance() const
    {
        return platoon[hostPosInPlatoon_].vehiclePosition;
    }

    // Return the current crosstrack distance, in m.
    double PlatoonManager::getCurrentCrosstrackDistance() const
    {
        return platoon[hostPosInPlatoon_].vehicleCrossTrack;
    }

    // UCLA: return the host vehicle static ID.
    std::string PlatoonManager::getHostStaticID() const
    {
        return HostMobilityId;
    }

    // Return the physical length from platoon front vehicle (front bumper) to platoon rear vehicle (rear bumper) in m.
    double PlatoonManager::getCurrentPlatoonLength() {
        //this works even if platoon size is 1 (can't be 0)
        return platoon[0].vehiclePosition - platoon[platoon.size() - 1].vehiclePosition + config_.vehicleLength; 
    }


    // ---------------------- UCLA: IHP platoon trajectory regulation --------------------------- //

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
        std::vector<double> downtrackDistance(platoon.size());
        for (size_t i = 0; i < platoon.size(); i++)
        {
            downtrackDistance[i] = platoon[i].vehiclePosition;
        }
        // speed vector
        std::vector<double> speed(platoon.size());
        for (size_t i = 0; i < platoon.size(); i++)
        {
            speed[i] = platoon[i].vehicleSpeed;
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

        // ---> 4.4 return IHP calculated desired speed
        return pos_des;
    }

    // UCLA: find the index of the closest vehicle that is in front of the host vehicle (cut-in joiner).
    // Note: The joiner will cut-in at the back of this vehcile, which make this index points to the vehicle that is leading the cut-in gap.
    int PlatoonManager::getClosestIndex(double joinerDtD)
    {   
        /*
            A naive way to find the closest member that is in front of the host that point to the gap to join the platoon.
            Note: The potential gap leading vehicle should be in front of the joiner (i.e., gap leading vehicle's dtd > joiner's dtd).
                  If the joiner is already in front of the platoon leader, this function will return -1 (i.e., cut-in front).
        */

        double min_diff = 99999.000;
        int cut_in_index = -1;
        // Loop through all platoon members  
        for(int i = 0; i < platoon.size(); i++) 
        {
            double current_member_dtd = platoon[i].vehiclePosition; 
            double curent_dtd_diff = current_member_dtd - joinerDtD;
            // update min index
            if (curent_dtd_diff > 0 && curent_dtd_diff < min_diff)
            {
                min_diff = current_member_dtd;
                cut_in_index = i;
            }
        }
        
        return cut_in_index;
    }

    // UCLA: find the cut-in join target gap size in downtrack distance (m). The origin of the vehicle when calculating DtD is locate at the rear axle. 
    double PlatoonManager::getCutInGap(int gap_leading_index, double joinerDtD)
    {
        /*
            Locate the target cut-in join gap size based on the index.   
        */
        // Initiate variables 
        double gap_size;

        // cut-in from front 
        if (gap_leading_index == -1)
        {
            double gap_rear_dtd = platoon[0].vehiclePosition;
            double gap_size = joinerDtD - gap_rear_dtd - config_.vehicleLength;
        }
        // cut-in from behind 
        else if (gap_leading_index == platoon.size() - 1)
        {    
            double gap_leading_dtd = platoon[gap_leading_index].vehiclePosition;
            double gap_size = gap_leading_dtd - joinerDtD - config_.vehicleLength;;
        }
        // cut-in in the middle
        else
        {
            double gap_leading_dtd = platoon[gap_leading_index].vehiclePosition;
            double gap_rear_dtd = platoon[gap_leading_index + 1].vehiclePosition;
            double gap_size = gap_leading_dtd - gap_rear_dtd - config_.vehicleLength;
        }

        // return gap_size value
        return gap_size;
    }
}