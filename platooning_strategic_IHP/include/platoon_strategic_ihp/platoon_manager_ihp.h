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


#include <carma_v2x_msgs/msg/mobility_operation.hpp>
#include <carma_v2x_msgs/msg/mobility_request.hpp>
#include <carma_v2x_msgs/msg/mobility_response.hpp>
#include <carma_v2x_msgs/msg/plan_type.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <carma_wm_ros2/WMListener.hpp>
#include <carma_wm_ros2/WorldModel.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <carma_ros2_utils/timers/TimerFactory.hpp>
#include "platoon_config_ihp.h"

namespace platoon_strategic_ihp
{
    /**
    * \brief Struct for an action plan, which describes a transient joining activity
    */ 
    struct ActionPlan 
    {
        bool            valid;          //is host currently negotiating/executing a join action?
        unsigned long   planStartTime;  //time that plan was initiated
        std::string     planId;         //ID of this action
        std::string     peerId;         //vehicle ID of the candidtate joining vehicle or leader of platoon we are joining

        ActionPlan() : valid(false), planStartTime(0), planId(""), peerId("") {}

        ActionPlan(bool valid, unsigned long planStartTime, std::string planId, std::string peerId): 
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
    * \brief Platoon member info 
    */
    struct PlatoonMember
    {
        // Static ID is permanent ID for each vehicle
        std::string staticId;
        // Vehicle real time command speed in m/s.
        double commandSpeed;
        // Actual vehicle speed in m/s.
        double vehicleSpeed;
        // Vehicle current downtrack distance on the current route in m.
        double vehiclePosition;
        // Vehicle current crosstrack distance on the current route in m.
        double vehicleCrossTrack;
        // The local time stamp when the host vehicle update any informations of this member.
        long   timestamp;

        PlatoonMember(): staticId(""), commandSpeed(0.0), vehicleSpeed(0.0), vehiclePosition(0.0), vehicleCrossTrack(0.0), timestamp(0) {} 

        PlatoonMember(std::string staticId, double commandSpeed, double vehicleSpeed, double vehiclePosition, double vehicleCrossTrack, long timestamp): staticId(staticId),
            commandSpeed(commandSpeed), vehicleSpeed(vehicleSpeed), vehiclePosition(vehiclePosition), vehicleCrossTrack(vehicleCrossTrack), timestamp(timestamp) {}
    };


    class PlatoonManager
    {
    public:

        /**
        * \brief Constructor
        * 
        * \param timer_factory An interface which can be used to get access to the current time
        */
        PlatoonManager(std::shared_ptr<carma_ros2_utils::timers::TimerFactory> timer_factory);

        /**
         * \brief Stores the latest info on location of the host vehicle.
         * 
         * \param downtrack distance downtrack from beginning of route, m
         * \param crosstrack distance crosstrack from roadway centerline, m
         */
        void updateHostPose(const double downtrack, const double crosstrack);

        /**
         * \brief Stores the latest info on host vehicle's command & actual speeds
         * 
         * \param cmdSpeed current commanded speed, m/s
         * \param actualSpeed current measured speed, m/s
         */
        void updateHostSpeeds(const double cmdSpeed, const double actualSpeed);

        /**
        * \brief Update information for members of the host's platoon based on a mobility operation STATUS message
        * 
        * \param senderId static id of the broadcasting vehicle
        * \param platoonId platoon id
        * \param params message strategy parameters
        * \param DtD downtrack distance along host's route, m
        * \param CtD crosstrack distance in roadway at host's location, m
        */
        void hostMemberUpdates(const std::string& senderId, const std::string& platoonId, const std::string& params,
                               const double& DtD, const double& CtD);

        /**
        * \brief Update information for members of a neighboring platoon based on a mobility operation STATUS message
        * 
        * \param senderId static id of the broadcasting vehicle
        * \param platoonId platoon id
        * \param params message strategy parameters
        * \param DtD downtrack distance along host's route, m
        * \param CtD crosstrack distance in roadway at host's location, m
        */
        void neighborMemberUpdates(const std::string& senderId, const std::string& platoonId, const std::string& params,
                                   const double& DtD, const double& CtD);

        /**
         * \brief Updates the list of vehicles in the specified platoon, based on info available from a
         * mobility operation STATUS message from one of that platoon's vehicles.  It ensures the list of vehicles
         * is properly sorted in order of appearance from front to rear in the platoon.  If the host is in the
         * platoon, it will update host info as well.
         * 
         * \param platoon the list of vehicles in the platoon in question
         * \param senderId vehicle ID that sent the current info
         * \param cmdSpeed the commanded speed of the sending vehicle
         * \param dtDistance the downtrack location (from beginning of host's route) of the sending vehicle, m
         * \param ctDistance the crosstrack location (from center of roadway at host's current route location) of sending vehicle, m
         * \param curSpeed the current actual speed of the sending vehicle, m/s
         **/
        void updatesOrAddMemberInfo(std::vector<PlatoonMember>& platoon, std::string senderId, double cmdSpeed,
                                    double dtDistance, double ctDistance, double curSpeed);
        
        /**
        * \brief Returns total size of the platoon , in number of vehicles.
        */
        int getHostPlatoonSize();

        /**
         * \brief Resets necessary variables to indicate that the current ActionPlan is dead.
         */
        void clearActionPlan();

        /**
         * \brief Resets all  variables that might indicate other members of the host's platoon; sets the host back to solo vehicle.
         */
        void resetHostPlatoon();

        /**
         * \brief Resets all variables that describe a neighbor platoon, so that no neighbor is known.
         */
        void resetNeighborPlatoon();

        /**
         * \brief Removes a single member from the internal record of platoon members
         * 
         * \param mem index of the member to be removed (zero-based)
         * 
         * \return true if removal was successful, false otherwise
         */
        bool removeMember(const size_t mem);

        /**
         * \brief Removes a single member from the internal record of platoon members
         * 
         * \param id the vehicle ID of the member to be removed
         * 
         * \return true if removal was successful, false otherwise
         */
        bool removeMemberById(const std::string id);
        
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
        * \param newPlatoonId new ID of the platoon
        * \param newLeaderId ID of the new lead vehicle
        */
        void changeFromLeaderToFollower(std::string newPlatoonId, std::string newLeaderId);
        
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
        double getCommandSpeed() const;

        /**
        * \brief Returns current downtrack distance, in m.
        */
        double getCurrentDowntrackDistance() const;

        /**
        * \brief Returns current crosstrack distance, in m.
        */
        double getCurrentCrosstrackDistance() const;

        /**
        * \brief Returns current host static ID as a string.
        */
        std::string getHostStaticID() const;

        /**
         * \brief UCLA: Return the platoon leader downtrack distance, in m.
         */
        double getPlatoonFrontDowntrackDistance();

        /**
         * \brief UCLA: Return the time headway summation of all predecessors, in s.
         */
        double getPredecessorTimeHeadwaySum();
        
        /**
         * \brief UCLA: Return the speed of the preceding vehicle, in m/s.
         */
        double getPredecessorSpeed();

        /**
         * \brief UCLA: Return the position of the preceding vehicle, in m.
         */
        double getPredecessorPosition();
        
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
         * \brief UCLA: Return joiner's desired position in terms of target platoon index to cut into the platoon. 
         *        CAUTION: ASSUMES that the neighbor platoon info is fully populated!
         * 
         * \param joinerDtD: The current downtrack distance (with regards to host vehicle) of the joiner vehicle.
         * 
         * \return: cut-in index: The index of the gap-leading vehicle within the platoon. If front join, return -1.
         */
        int getClosestIndex(double joinerDtD);

        /**
         * \brief UCLA: Return the current actual gap size in the target platoon for cut-in join, in m.
         *        Note: The origin of the vehicle (for downtrack distance calculation) is located at the rear axle.
         *        CAUTION: ASSUMES that the neighbor platoon info is fully populated!
         * 
         * \param gap_leading_index: The platoon index of the  gap-leading vehicle. 
         * \param joinerDtD: The current downtrack distance (with regards to host vehicle route) of the joiner vehicle.
         * 
         * \return: cut-in gap: The desired gap size for cut-in join, in m.
         */
        double getCutInGap(const int gap_leading_index, const double joinerDtD);

        const std::string dummyID = "00000000-0000-0000-0000-000000000000";

        // List of members in the host's own platoon (host will always be represented, so size is never zero)
        std::vector<PlatoonMember> host_platoon_;

        // Platoon ID of the host's platoon
        std::string currentPlatoonID = dummyID; //dummy indicates not part of a platoon

        // Vehicle ID of the host's platoon leader (host may be the leader)
        std::string platoonLeaderID = dummyID;  //dummy indicates not part of a platoon

        // Current platooning state of the host vehicle
        PlatoonState current_platoon_state = PlatoonState::STANDBY;

        //index to the host_platoon_ vector that represents the host vehicle
        size_t hostPosInPlatoon_ = 0;

         // Plan that represents a joining activity only, it is NOT the ID of the platoon itself
        ActionPlan current_plan = ActionPlan();

        // Is the host a follower in its platoon?
        bool isFollower = false;

        // List of members in a detected neighbor platoon, which may be empty
        // CAUTION: we can only represent one neighbor platoon in this version, so if multiple platoons are nearby,
        //          code will get very confused and results are unpredictable.
        std::vector<PlatoonMember> neighbor_platoon_;

        // Num vehicles in the neighbor platoon, as indicated by the size field in the INFO message
        size_t neighbor_platoon_info_size_ = 0;

        // Platoon ID of the neighboring platoon
        std::string targetPlatoonID = dummyID;  //ID of a real platoon that we may be attempting to join

        std::string neighborPlatoonID = dummyID;  //ID of the neighbor platoon that we may be attempting to join (dummy if neighbor is a solo vehicle)

        // Vehicle ID of the neighbor platoon's leader
        std::string neighbor_platoon_leader_id_ = dummyID; //dummy indicates unknown

        // Is the record of neighbor platoon members complete (does it contain a record for every member)?
        bool is_neighbor_record_complete_ = false;

        // Current vehicle pose in map
        geometry_msgs::msg::PoseStamped pose_msg_;

        // host vehicle's static ID 
        std::string HostMobilityId = "default_host_id";

        // UCLA: add indicator of creating gap
        bool isCreateGap = false;

        size_t dynamic_leader_index_ = 0;

        // Timer factory used to get current time
        std::shared_ptr<carma_ros2_utils::timers::TimerFactory> timer_factory_;
        
    private:

        // local copy of configuration file
        PlatoonPluginConfig config_;

        std::string OPERATION_INFO_TYPE = "INFO";
        std::string OPERATION_STATUS_TYPE = "STATUS";
        std::string JOIN_AT_REAR_PARAMS = "SIZE:%1%,SPEED:%2%,DTD:%3%";
        // UCLA: add params for frontal join
        std::string JOIN_FROM_FRONT_PARAMS = "SIZE:%1%,SPEED:%2%,DTD:%3%";
        std::string  MOBILITY_STRATEGY = "Carma/Platooning";

        double minCutinGap_ = 22.0;                                  // m
        double maxCutinGap_ = 32.0;                                  // m
        std::string previousFunctionalDynamicLeaderID_ = "";
        int previousFunctionalDynamicLeaderIndex_ = -1;

       // note: APF related parameters are in config.h.

        double vehicleLength_ = 5.0;                            // the length of the vehicle, in m.
        double gapWithPred_ = 0.0;                              // time headway with predecessor, in s.
        double downtrack_progress_ = 0;                         // current downtrack distance, in m.

        // Note: Parameters for IHP platoon trajectory regulation are in config.h. 

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