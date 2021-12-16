#pragma once

/*
 * Copyright (C) 2021 LEIDOS.
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
#include <cav_msgs/MobilityOperation.h>
#include <cav_msgs/MobilityRequest.h>
#include <cav_msgs/MobilityResponse.h>
#include <cav_msgs/PlanType.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <carma_wm/WMListener.h>
#include <carma_wm/WorldModel.h>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <autoware_msgs/ControlCommandStamped.h>
#include <boost/format.hpp>




namespace platoon_strategic
{
    /**
    * \brief Struct for a platoon plan
    */ 
    struct PlatoonPlan {
        
        bool valid;
        long planStartTime;
        std::string planId;
        std::string peerId;
        PlatoonPlan():valid(false), planStartTime(0), planId(""), peerId("") {} ;
        PlatoonPlan(bool valid, long planStartTime, std::string planId, std::string peerId): 
            valid(valid), planStartTime(planStartTime), planId(planId), peerId(peerId) {}  
    };

    /**
    * \brief A response to an MobilityRequest message.
    * ACK - indicates that the plugin accepts the MobilityRequest and will handle making any adjustments needed to avoid a conflict
    * NACK - indicates that the plugin rejects the MobilityRequest and would suggest the other vehicle replan
    * NO_RESPONSE - indicates that the plugin is indifferent but sees no conflict
    */
    enum MobilityRequestResponse {
            ACK,
            NACK,
            NO_RESPONSE
    };

    /**
    * \brief Platoon States
    */
    enum PlatoonState{
        STANDBY,
        LEADERWAITING,
        LEADER,
        CANDIDATEFOLLOWER,
        FOLLOWER
    };

    /**
    * \brief Platoon States
    */
    struct PlatoonMember{
        // Static ID is permanent ID for each vehicle
        std::string staticId;
        // Current BSM Id for each CAV
        std::string bsmId;
        // Vehicle real time command speed in m/s
        double commandSpeed;
        // Actual vehicle speed in m/s
        double vehicleSpeed;
        // Vehicle current down track distance on the current route in m
        double vehiclePosition;
        // The local time stamp when the host vehicle update any informations of this member
        long   timestamp;
        PlatoonMember(): staticId(""), bsmId(""), commandSpeed(0.0), vehicleSpeed(0.0), vehiclePosition(0.0), timestamp(0) {} 
        PlatoonMember(std::string staticId, std::string bsmId, double commandSpeed, double vehicleSpeed, double vehiclePosition, long timestamp): staticId(staticId),
            bsmId(bsmId), commandSpeed(commandSpeed), vehicleSpeed(vehicleSpeed), vehiclePosition(vehiclePosition), timestamp(timestamp) {}
        };
        


    class PlatoonManager
    {
    public:

        /**
        * \brief Default constructor
        */
        PlatoonManager();

        // Platoon List (initialized empty)
        std::vector<PlatoonMember> platoon{};

        

        // Current vehicle pose in map
        geometry_msgs::PoseStamped pose_msg_;

        // Current vehicle downtrack
        double current_downtrack_distance_ = 0;

        /**
        * \brief Update platoon members information
        * 
        * \param senderId static id of the broadcasting vehicle
        * \param platoonId platoon id
        * \param senderBsmId bsm id of the broadcasting vehicle
        * \param params strategy parameters
        * \param Dtd downtrack distance
        */
        void memberUpdates(const std::string& senderId,const std::string& platoonId,const std::string& senderBsmId,const std::string& params, const double& DtD);

        /**
         * Given any valid platooning mobility STATUS operation parameters and sender staticId,
         * in leader state this method will add/updates the information of platoon member if it is using
         * the same platoon ID, in follower state this method will updates the vehicle information who
         * is in front of the subject vehicle or update platoon id if the leader is join another platoon
         * \param senderId sender ID for the current info
         * \param platoonId sender platoon id
         * \param senderBsmId sender BSM ID
         * \param params strategy params from STATUS message in the format of "CMDSPEED:xx,DOWNTRACK:xx,SPEED:xx"
         **/
        void updatesOrAddMemberInfo(std::string senderId, std::string senderBsmId, double cmdSpeed, double dtDistance, double curSpeed);
        
        /**
        * \brief Returns total size of the platoon
        */
        int getTotalPlatooningSize() const;


        /**
        * \brief Returns leader of the platoon
        */
        PlatoonMember getLeader();

        /**
         * This is the implementation of all predecessor following (APF) algorithm for leader
         * selection in a platoon. This function will recognize who is acting as the current leader
         * of the subject vehicle. The current leader of the subject vehicle will be any ONE of
         * the vehicles in front of it. Having a vehicle further downstream function as the leader
         * is more efficient and more stable; however, having a vehicle closer to the subject vehicle
         * function as the leader is safer. For this reason, the subject vehicle will monitor
         * all time headways between every single set of consecutive vehicles starting from itself
         * to the leader. If the time headways are within some safe thresholds then vehicles further
         * downstream may function as the leader. Otherwise, for the sake of safety, vehicles closer
         * to the subject vehicle, potentially even the predecessor, will function as the leader.
         * @return the index of the leader in the platoon list
         */

        int allPredecessorFollowing();

        /**
        * \brief Update status when state change from Follower to Leader
        */
        void changeFromFollowerToLeader();
        
        /**
        * \brief Update status when state change from Leader to Follower
        *
        * \param newPlatoonId platoon id of the leader
        */
        void changeFromLeaderToFollower(std::string newPlatoonId);
        
        /**
        * \brief Get number of vehicles in front of host vehicle in platoon
        */
        int getNumberOfVehicleInFront();
        
        /**
        * \brief Returns Length of the platoon
        */
        double getCurrentPlatoonLength();

        /**
        * \brief Returns downtrack distance of the rear vehicle in platoon
        */
        double getPlatoonRearDowntrackDistance();
        
        /**
        * \brief Returns distance from start of the route
        */
        double getDistanceFromRouteStart() const;
        
        /**
        * \brief Returns distance to the front vehicle
        */
        double getDistanceToFrontVehicle();
        
        /**
        * \brief Returns current speed
        */
        double getCurrentSpeed() const;
        
        /**
        * \brief Returns command speed
        */
        double getCommandSpeed();

        /**
        * \brief Returns current downtrack distance
        */
        double getCurrentDowntrackDistance() const;

        // Member variables
        int platoonSize = 2;
        std::string leaderID = "default_leader_id";
        std::string currentPlatoonID = "default_test_id";
        bool isFollower = false;

        double current_speed_ = 0;
        double command_speed_ = 0;

        // Platoon State
        PlatoonState current_platoon_state = PlatoonState::STANDBY;
        
        // Platooning PLan
        PlatoonPlan current_plan;

        std::string targetLeaderId = "default_target_leader_id";

        std::string HostMobilityId = "default_host_id";


    private:

        std::string targetPlatoonId;
        std::string OPERATION_INFO_TYPE = "INFO";
        std::string OPERATION_STATUS_TYPE = "STATUS";
        std::string JOIN_AT_REAR_PARAMS = "SIZE:%1%,SPEED:%2%,DTD:%3%";
        std::string  MOBILITY_STRATEGY = "Carma/Platooning";
    

        double minGap_ = 22.0;
        double maxGap_ = 32.0;
        std::string previousFunctionalLeaderID_ = "";
        int previousFunctionalLeaderIndex_ = -1;

        double maxSpacing_ = 4.0;
        double minSpacing_ = 3.9;
        double lowerBoundary_ = 1.6;
        double upperBoundary_ = 1.7 ;

        double vehicleLength_ = 5.0;  // m

        double gapWithFront_ = 0.0;

        double downtrack_progress_ = 0;


        std::string algorithmType_ = "APF_ALGORITHM";

        bool insufficientGapWithPredecessor(double distanceToFrontVehicle);
        
        std::vector<double> calculateTimeHeadway(std::vector<double> downtrackDistance, std::vector<double> speed) const;
        int determineLeaderBasedOnViolation(std::vector<double> timeHeadways);

        // helper method for APF algorithm
        int findLowerBoundaryViolationClosestToTheHostVehicle(std::vector<double> timeHeadways) const;

        // helper method for APF algorithm
        int findMaximumSpacingViolationClosestToTheHostVehicle(std::vector<double> timeHeadways) const;

        std::vector<double> getTimeHeadwayFromIndex(std::vector<double> timeHeadways, int start) const;


    

    

    };
}