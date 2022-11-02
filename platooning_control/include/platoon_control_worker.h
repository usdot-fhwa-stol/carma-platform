
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





namespace platoon_control
{
    /**
    * \brief Platoon Leader Struct
    */
	struct PlatoonLeaderInfo{
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
            // leader index in the platoon
            int leaderIndex;
            // Number of vehicles in front
            int NumberOfVehicleInFront;

        };


        // Leader info: platoonmember + leader index + number of vehicles in front

    /**
    * \brief This is the worker class for platoon controller. It is responsible for generating and smoothing the speed and steering control commands from trajectory points.  
    */
    class PlatoonControlWorker
    {
    public:

        /**
        * \brief Default constructor for platooning control worker
        */
        PlatoonControlWorker();

        /**
        * \brief Update configurations
        * \param new_config new configuration
        */
        void updateConfigParams(PlatooningControlPluginConfig new_config);

        /**
        * \brief Returns latest speed command
        * \return lastest speed command in m/s
        */
        double getLastSpeedCommand() const;

        /**
        * \brief Generates speed commands (in m/s) based on the trajectory point
        * \param point trajectory point
        */
        void generateSpeed(const cav_msgs::TrajectoryPlanPoint& point);
        
        /**
        * \brief Generates steering commands (in rad) based on lookahead trajectory point
        * \param point trajectory point
        */
        void generateSteer(const cav_msgs::TrajectoryPlanPoint& point);

        /**
        * \brief Sets the platoon leader object using info from msg
        * \param leader leader information msg received from strategic plugin
        */
        void setLeader(const PlatoonLeaderInfo& leader);
        
        /**
        * \brief set current pose
        * \param msg pose value
        */
        void setCurrentPose(const geometry_msgs::PoseStamped msg);


        /**
        * \brief set current speed
        * \param speed speed value
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

		// geometry pose
        geometry_msgs::Pose current_pose_;


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
