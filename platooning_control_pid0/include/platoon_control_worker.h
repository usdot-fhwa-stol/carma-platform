
/*------------------------------------------------------------------------------
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
------------------------------------------------------------------------------*/

#include <ros/ros.h>
#include <cav_msgs/MobilityOperation.h>
#include <cav_msgs/MobilityRequest.h>
#include <cav_msgs/MobilityResponse.h>
#include <geometry_msgs/PoseStamped.h>
#include <cav_msgs/PlanType.h>
#include "pid_controller.h"
#include "pure_pursuit.h"
#include "platoon_control_config.h"
#include <boost/optional.hpp>


namespace platoon_control_pid0
{
    /**
    * \brief Platoon Leader Struct
    */
	struct PlatoonLeaderInfo{
        // Static ID is permanent ID for each vehicle
        std::string staticId;
        // Vehicle real time command speed in m/s
        double commandSpeed;
        // Actual vehicle speed in m/s
        double vehicleSpeed;
        // Vehicle current down track distance on the current route in m
        double vehiclePosition;
        // The local time stamp when the host vehicle update any informations of this member
        long   timestamp;
        // leader index in the platoon
        int leaderIndex;
        // Number of vehicles in front
        int NumberOfVehicleInFront;
    };


    class PlatoonControlWorker
    {
    public:

        /**
        * \brief Default constructor for platoon control worker
        */
        PlatoonControlWorker();

        /**
         * \brief Destructor for platoon control worker
         */
        ~PlatoonControlWorker();

        /**
         * \brief Stores the needed config params for this plugin
         */
        void set_config_params(PlatoonControlPluginConfig configs);

        /**
         * \brief Stores the current vehicle pose (x-y-z location in map frame, m)
         */
        void set_current_pose(const geometry_msgs::PoseStamped p);

        /**
         * \brief Stores the current vehicle forward speed, m/s
         */
        void set_current_speed(const double speed);

        /**
         * \brief Stores the current trajectory plan
         */
        void set_trajectory(const cav_msgs::TrajectoryPlan::ConstPtr& tp);

        /**
         * \brief Stores info about the current platoon situation
         * \param leader_id         ID of the host's dynamic leader (the one we are controlling gap to)  //TODO: needed?
         * \param leader_loc        leader's downtrack distance relative to host's route, m
         * \param leader_spd_cmd    speed command the dynamic leader is trying to follow, m/s
         * \param leader_pos        position of the dynamic leader in the platoon (zero-indexed)
         * \param host_pos          position of the host vehicle in the platon (zero-indexed)
         */
        void set_platoon_info(const std::string leader_id, const double leader_loc, const double leader_spd_cmd,
                              const int leader_pos, const int host_pos);

        /**
         * \brief Returns the newly calculated speed command, m/s
         */
        double get_speed_cmd();

        /**
         * \brief Returns the newly calculated angular velocity command, rad/s
         */
        double get_angular_vel_cmd();

        /**
         * \brief Returns the newly calculated steering angle command, rad
         */
        double get_steering_cmd();

        /**
         * \brief Performs calculations to generate the three required command outputs:
         * speed command, angular rate command, steering command
         */
        void generate_control_signal();


    private:

        PIDController *     pid_h_ = nullptr;               //PID controller for heading error
        PIDController *     pid_c_ = nullptr;               //PID controller for cross-track error

        double              time_step_;                     //time between control loop iterations, s
        geometry_msgs::Pose current_pose_;                  //current location of the host vehicle, m offsets in map frame
        double              current_speed_;                 //current host forward speed, m/s
        std::vector<cav_msgs::TrajectoryPlanPoint> traj_;   //the set of points forming the current trajectory
        std::string         leader_id_;                     //ID of the dynamic leader for this host
        double              leader_loc_;                    //downtrack distance of the leader wrt host's route, m
        double              leader_spd_cmd_;                //speed command that the leader is controlling to, m/s
        int                 leader_pos_;                    //position of the dynamic leader in the platoon (zero-indexed)
        int                 host_pos_;                      //position of the host vehicle in the platoon (zero-indexed)
        double              gamma_h_;                       //steering command mixing ratio between heading & CTE PIDs
        int                 tp_index_;                      //index of the nearest trajectory point in front of vehicle


        /**
         * \brief Given a sorted array of trajectory points (indicating direction of travel), finds the
         *          nearest point to the vehicle that is still in front of the vehicle.  It is assumed that
         *          trajectories will be long enough and populated frequently enough that they will always
         *          be close to the vehicle, so that a meaningfully close point will always be found.
         *          Stores resulting identified point index in member variable tp_index_.
         */
        void find_nearest_point();

        /**
         * \brief Returns the smallest delta angle between two heading values, accounting for the possibility that
         *          they may be on opposite sides of the zero cardinal heading.
         * \param h1 first heading, rad east of north in [0, 2*pi)
         * \param h2 second heading, rad east of north in [0, 2*pi)
         * \return delta angle, rad in (-pi, pi], positive if h1 > h2
         */
        double subtract_headings(const double h1, const double h2);

        /**
         * \brief Calculates the distance from the vehicle's origin to the trajectory path, moving
         *          perpendicularly to the vehicle's heading.
         * \return cross-track error, m, with positive values being left of trajectory
         */
        double calculate_cross_track();





        /**
        * \brief Update configurations
        */
        void updateConfigParams(PlatooningControlPluginConfig new_config);

        /**
        * \brief Returns latest speed command
        */
        double getLastSpeedCommand() const;

        /**
        * \brief Generates speed commands based on the trajectory point
        */
        void generateSpeed(const cav_msgs::TrajectoryPlanPoint& point);
        
        /**
        * \brief Generates steering commands based on lookahead trajectory point
        */
        void generateSteer(const cav_msgs::TrajectoryPlanPoint& point);

        /**
        * \brief set platoon leader
        */
        void setLeader(const PlatoonLeaderInfo& leader);
        
        /**
        * \brief set current speed
        */
        void setCurrentSpeed(double speed);

        // Member Variables
        double speedCmd = 0;
        double currentSpeed = 0;
        double lastCmdSpeed = 0.0;
        double speedCmd_ = 0;
        double steerCmd_ = 0;
        double angVelCmd_ = 0;
        double desired_gap_ = ctrl_config_.standStillHeadway;
        double actual_gap_ = 0.0;
        bool last_cmd_set_ = false;

        // Platoon Leader
        PlatoonLeaderInfo platoon_leader;


        void setCurrentPose(const geometry_msgs::PoseStamped msg);

		// geometry pose


    private:
        // config parameters
        PlatooningControlPluginConfig ctrl_config_;

        // pid controller object
        PIDController pid_ctrl_;

        // pure pursuit controller object
        PurePursuit pp_;

        double dist_to_front_vehicle;

        bool leaderSpeedCapEnabled = true;
        bool enableMaxAdjustmentFilter = true;
        
        bool speedLimitCapEnabled = true;
        bool enableLocalSpeedLimitFilter = true;
        
        bool maxAccelCapEnabled = true;
        bool enableMaxAccelFilter = true;
        

    };
}
