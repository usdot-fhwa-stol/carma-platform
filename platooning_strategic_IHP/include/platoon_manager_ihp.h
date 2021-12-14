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

/*
 * Developed by the UCLA Mobility Lab, 10/20/2021. 
 *
 * Creator: Xu Han
 * Author: Xu Han, Xin Xia, Jiaqi Ma
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




namespace platoon_strategic_ihp
{
    /**
    * \brief Struct for a platoon plan
    */ 
    struct PlatoonPlan 
    {
        
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
    enum MobilityRequestResponse 
    {
            ACK,
            NACK,
            NO_RESPONSE
    };

    /**
    * \brief Platoon States. UCLA: Added additional states 
    * (i.e., LEADERABORTING and CANDIDATELEADER) for same-lane front join.
    * (i.e., LEADWITHOPERATION and PREPARETOJOIN) for cut-in front join.
    *  
    */
    enum PlatoonState
    {
        STANDBY,                    // 0;
        LEADERWAITING,              // 1;
        LEADER,                     // 2;
        CANDIDATEFOLLOWER,          // 3;
        FOLLOWER,                   // 4;
        //UCLA: FRONTAL JOIN STATE
        LEADERABORTING,             // 5;
        //UCLA: FRONTAL JOIN STATE
        CANDIDATELEADER,            // 6;
        //UCLA: CUT-IN JOIN STATE
        LEADWITHOPERATION,          // 7;
        //UCLA: CUT-IN JOIN STATE
        PREPARETOJOIN               // 8;
    };

    /**
    * \brief Platoon States
    */
    struct PlatoonMember
    {
        // Static ID is permanent ID for each vehicle
        std::string staticId;
        // Vehicle real time command speed in m/s.
        double commandSpeed;
        // Actual vehicle speed in m/s.
        double vehicleSpeed;
        // Vehicle current down track distance on the current route in m.
        double vehiclePosition;
        // The local time stamp when the host vehicle update any informations of this member.
        long   timestamp;
        PlatoonMember(): staticId(""), commandSpeed(0.0), vehicleSpeed(0.0), vehiclePosition(0.0), timestamp(0) {} 
        PlatoonMember(std::string staticId, double commandSpeed, double vehicleSpeed, double vehiclePosition, long timestamp): staticId(staticId),
            commandSpeed(commandSpeed), vehicleSpeed(vehicleSpeed), vehiclePosition(vehiclePosition), timestamp(timestamp) {}
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
        void memberUpdates(const std::string& senderId, const std::string& platoonId, const std::string& params, const double& DtD);

        /**
         * \brief Given any valid platooning mobility STATUS operation parameters and sender staticId,
         * in leader state this method will add/updates the information of platoon member if it is using
         * the same platoon ID, in follower state this method will updates the vehicle information who
         * is in front of the subject vehicle or update platoon id if the leader is join another platoon
         * 
         * \param senderId sender ID for the current info
         * \param platoonId sender platoon id
         * \param params strategy params from STATUS message in the format of "CMDSPEED:xx,DOWNTRACK:xx,SPEED:xx"
         **/
        void updatesOrAddMemberInfo(std::string senderId, double cmdSpeed, double dtDistance, double curSpeed);
        
        /**
        * \brief Returns total size of the platoon , in number of vehicles.
        */
        int getTotalPlatooningSize();


        /**
        * \brief Returns dynamic leader of the host vehicle.
        * 
        * \return The current dynamic leader as a vehcile object. 
        */
        PlatoonMember getDynamicLeader();


        /**
         * \brief This is the implementation of all predecessor following (APF) algorithm for leader
         * selection in a platoon. This function will recognize who is acting as the current leader
         * of the subject vehicle. The current leader of the subject vehicle will be any ONE of
         * the vehicles in front of it. Having a vehicle further downstream function as the leader
         * is more efficient and more stable; however, having a vehicle closer to the subject vehicle
         * function as the leader is safer. For this reason, the subject vehicle will monitor
         * all time headways between every single set of consecutive vehicles starting from itself
         * to the leader. If the time headways are within some safe thresholds then vehicles further
         * downstream may function as the leader. Otherwise, for the sake of safety, vehicles closer
         * to the subject vehicle, potentially even the predecessor, will function as the leader.
         * 
         * \return the index of the leader in the platoon list.
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
        * \brief Get number of vehicles in front of host vehicle inside platoon
        */
        int getNumberOfVehicleInFront();
        
        /**
        * \brief Returns overall length of the platoon. in m.
        */
        double getCurrentPlatoonLength();

        /**
        * \brief Returns downtrack distance of the rear vehicle in platoon, in m.
        */
        double getPlatoonRearDowntrackDistance();
        
        /**
        * \brief Returns distance to the predessecor vehicle, in m.
        */
        double getDistanceToPredVehicle();
        
        /**
        * \brief Returns current speed, in m/s.
        */
        double getCurrentSpeed() const;
        
        /**
        * \brief Returns command speed. in m/s.
        */
        double getCommandSpeed();

        /**
        * \brief Returns current downtrack distance, in m.
        */
        double getCurrentDowntrackDistance() const;

        /**
        * \brief Returns current host static ID as a string.
        */
        std::string getHostStaticID() const;

        /**
         * \brief UCLA: Return the platoon leader downtrack distance, in m.
         */
        double getPlatoonFrontDowntrackDistance();

        /**
        * \brief UCLA: Return follower's desired position (i.e., downtrack, in m) that maintains the desired 
        * intra-platoon time gap, based on IHP platoon trajectory regulation algorithm.
        * 
        * \params dt: The planning time step, in s.
        * 
        * \return: Host vehicle's desired position as downtrack distance, in m.
        */
        double getIHPDesPosFollower(double dt);

        /**
        \brief UCLA: Return joiner's desired position
        to cut into the platoon 
        */
        int getClosestIndex(double joinerDtD);

        /**
        \brief UCLA: Return the desired gap size
        for cut-in join 
        */
        double getCutInGap(int gap_leading_index, double joinerDtD);

        // Member variables
        int platoonSize = 2;
        std::string leaderID = "default_leader_id";
        std::string currentPlatoonID = "default_test_id";
        bool isFollower = false;

        double current_speed_ = 0; // m/s
        double command_speed_ = 0; // m/s

        // Platoon State
        PlatoonState current_platoon_state = PlatoonState::STANDBY;
        
        // Platooning PLan
        PlatoonPlan current_plan;

        std::string targetLeaderId = "default_target_leader_id";

        // host vehicle's static ID 
        std::string HostMobilityId = "default_host_id";

        // UCLA: add indicator of creating gap
        bool isCreateGap = false;
        // UCLA: add indicator of lane change 
        bool safeToLaneChange = false;
        
    private:

        // local copy of configuration file
        PlatoonPluginConfig config_;

        std::string targetPlatoonId;
        std::string OPERATION_INFO_TYPE = "INFO";
        std::string OPERATION_STATUS_TYPE = "STATUS";
        std::string JOIN_AT_REAR_PARAMS = "SIZE:%1%,SPEED:%2%,DTD:%3%";
        // UCLA: add params for frontal join
        std::string JOIN_FROM_FRONT_PARAMS = "SIZE:%1%,SPEED:%2%,DTD:%3%";
        std::string  MOBILITY_STRATEGY = "Carma/Platooning";

        double minGap_ = 22.0;                                  // m
        double maxGap_ = 32.0;                                  // m
        std::string previousFunctionalDynamicLeaderID_ = "";
        int previousFunctionalDynamicLeaderIndex_ = -1;

        // note: APF related parameters are moved to config.h.

        double vehicleLength_ = 5.0;                            // the length of the vehicle, in m.
        double gapWithPred_ = 0.0;                              // time headway with predecessor, in s.
        double downtrack_progress_ = 0;                         // current downtrack distance, in m.

        // Note: Parameters for IHP platoon trajectory regulation are moved to config.h. 

        std::string algorithmType_ = "APF_ALGORITHM";           // a string that defines the current algorithm to determine the dynamic leader.

        /**
        * \brief Check the gap with the predecessor vehicle. 
        * 
        * \param distanceToPredVehicle: The distance between the host to the predecessor vehicle. 
        * 
        * \return (bool) if the predecessor is to close.
        */
        bool insufficientGapWithPredecessor(double distanceToPredVehicle);
                
        /**
        * \brief Calculate the time headaway of each platoon member and save as a vector.
        * 
        * \param downtrackDistance: a vector containing the downtrack distances of all members.
        * \param speed: a vector containing the speeds of all members.
        * 
        * \return A vector containing the time headaway of all platoon members, each headway is in s.
        */
        std::vector<double> calculateTimeHeadway(std::vector<double> downtrackDistance, std::vector<double> speed) const;
        
        /**
        * \brief Determine the proper vehicle to follow based the time headway of each member. 
        * Note that the host will always choose the closest violator (i.e. time headaway too small or too large) to follow.
        * 
        * \param timeHeadways: A vector containing the time headaway of all platoon members.
        * 
        * \return An index indicating the proper vehicle to follow (i.e., leader). If choose to follow platoon leader, return 0.
        */
        int determineDynamicLeaderBasedOnViolation(std::vector<double> timeHeadways);
        
        /**
        * \brief Find the closest vehicle to the host vehicle that violates the (time headaway) lower boundary condition. 
        * 
        * \param timeHeadways: A vector containing the time headaway of all platoon members.
        * 
        * \return An index indicating the closest violating vehicle. If no violator, return -1.
        */
       int findLowerBoundaryViolationClosestToTheHostVehicle(std::vector<double> timeHeadways) const;

        /**
        * \brief Find the closest vehicle to the host vehicle that violates the (time headaway) maximum spacing condition. 
        * 
        * \param timeHeadways: A vector containing the time headaway of all platoon members.
        * 
        * \return An index indicating the closest violating vehicle. If no violator, return -1.
        */
        int findMaximumSpacingViolationClosestToTheHostVehicle(std::vector<double> timeHeadways) const;

        /**
        * \brief Return a sub-vector of the platoon-wise time headaways vector that start with a given index.  
        * 
        * \param timeHeadways: A vector containing the time headaway of all platoon members.
        *        start: An integer indicates the starting position.
        * 
        * \return An sub-vector start with given index.
        */
        std::vector<double> getTimeHeadwayFromIndex(std::vector<double> timeHeadways, int start) const;
    

    };
}