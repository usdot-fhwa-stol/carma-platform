
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

#include <boost/optional.hpp>
#include <ros/ros.h>
#include <cav_msgs/MobilityOperation.h>
#include <cav_msgs/MobilityRequest.h>
#include <cav_msgs/MobilityResponse.h>
#include <cav_msgs/TrajectoryPlan.h>
#include <geometry_msgs/PoseStamped.h>
#include <cav_msgs/PlanType.h>
#include "pid_controller.h"
#include "platoon_control_config.h"


namespace platoon_control_pid0
{
    /**
    * \brief Platoon Leader Struct
    */
	struct DynamicLeaderInfo {
        // Static ID is permanent ID for each vehicle
        std::string staticId;

        // Vehicle real time command speed in m/s
        double      commandSpeed;

        // Vehicle current down track distance on the current route in m
        double      vehiclePosition;
   };


    class PlatoonControlWorker {

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
        void set_config_params(const PlatoonControlPluginConfig config);

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
         * \brief Stores info needed for forward gap management.
         * \param pl A struct containing data about the forward vehicle whose speed host is trying to control to
         * \param tgt_gap The distance we would like to be from that forward vehicle, m
         * \param act_gap The distance we actually are from that forward vehicle, m
         */
        void set_lead_info(const DynamicLeaderInfo& dl, const double tgt_gap, const double act_gap);

        /**
         * \brief Performs calculations to generate the three required command outputs:
         * speed command, angular rate command, steering command
         */
        void generate_control_signal();

        /**
         * \brief Returns the newly calculated speed command, m/s
         */
        double get_speed_cmd();

        /**
         * \brief Returns the newly calculated steering angle command, rad
         */
        double get_steering_cmd();

        /**
         * \brief Returns the newly calculated angular velocity command, rad/s
         */
        double get_angular_vel_cmd();

        /**
         * \brief Returns the index of the nearest trajectory point downtrack of the vehicle
         */
        size_t get_tp_index();

        /**
         * SHOULD BE PRIVATE!
         * \brief Given a sorted array of trajectory points (indicating direction of travel), finds the
         *          nearest point to the vehicle that is in front of the vehicle.  It is assumed that
         *          trajectories will be long enough and populated frequently enough that they will always
         *          be close to the vehicle, so that a meaningfully close point will always be found.
         *          Stores resulting identified point index in member variable tp_index_.
         */
        void find_nearest_point();


        /**
         * SHOULD BE PRIVATE!
         * \brief Calculates the distance from the vehicle's origin to the trajectory path, moving
         *          perpendicularly to the vehicle's heading.
         * \return cross-track error, m, with positive values being left of trajectory
         */
        double calculate_cross_track();


        /** SHOULD BE PRIVATE!
         * \brief Calculates the heading we want to steer to, based on the given trajectory plan.  If a heading_lookahead
         *          is specified, then it will consider the heading that number of TPs in front of the vehicle as well as
         *          the nearest TP.  It then uses whichever of these has the largest magnitude difference from current
         *          vehicle heading as the result.
         * \return desired heading, rad in [0, 2pi)
         */
        double calc_desired_heading();


        /**
         * \brief FOR UNIT TESTING ONLY - allows direct injection of relevant pose data
         */
        void unit_test_set_pose(const double x, const double y, const double heading);

        /**
         * \brief FOR UNIT TESTING ONLY - allows direct injection of a sample trajectory
         */
        void unit_test_set_traj(const std::vector<cav_msgs::TrajectoryPlanPoint> tr);

        double unit_test_get_traj_px(const size_t index);

        double unit_test_get_traj_py(const size_t index);

        void unit_test_set_heading_lookahead(const int lookahead);


    private:

        PIDController *     pid_h_ = nullptr;               //PID controller for heading error
        PIDController *     pid_c_ = nullptr;               //PID controller for cross-track error

        double              time_step_;                     //time between control loop iterations, s
        double              gamma_h_;                       //steering command mixing ratio between heading & CTE PIDs
        int                 heading_lookahead_;             //num TPs downtrack to consider for determining trajectory heading
        double              wheelbase_;                     //wheelbase of the vehicle, m
        double              max_steering_angle_;            //upper limit on steering angle, rad
        double              heading_bias_;                  //fixed error in vehicle heading measurement that can't be steered out, rad
        double              host_x_;                        //current x coordinate of host vehicle location, m offset in map frame
        double              host_y_;                        //current y coordinate of host vehicle location, m offset in map frame
        double              host_heading_;                  //current heading angle of host vehicle, rad N of E in [0, 2pi)
        double              current_speed_;                 //current host forward speed, m/s
        std::vector<cav_msgs::TrajectoryPlanPoint> traj_;   //the set of points forming the current trajectory
        size_t              tp_index_;                      //index of the nearest trajectory point in front of vehicle
        DynamicLeaderInfo   leader_;                        //holds data about the dynamic leader that host is trying to follow
        int                 host_pos_;                      //position of the host vehicle in the platoon (zero-indexed)
        double              desired_gap_;                   //distance to dynamic leader that we would like to have, m
        double              actual_gap_;                    //actual distance to dynamic leader, m
        double              steering_cmd_;                  //output command for front wheel steering angle, rad (left is positive)
        double              angular_vel_cmd_;               //output command for angular velocity of the vehicle, rad/s (around +Z axis)
        double              speed_cmd_;                     //output command for forward speed, m/s


        /**
         * \brief Returns the smallest delta angle between two heading values, accounting for the possibility that
         *          they may be on opposite sides of the zero cardinal heading.
         * \param h1 first heading, rad north of east in [0, 2*pi)
         * \param h2 second heading, rad north of east in [0, 2*pi)
         * \return delta angle, rad in (-pi, pi], positive if h1 > h2
         */
        double subtract_headings(const double h1, const double h2);

        /**
         * \brief Normalizes the given yaw angle to the range [0, 2pi).
         * \param yaw the raw yaw angle, rad, which could be outside the desired range on either side.
         * \return equivalent angle, rad in [0, 2pi)
         */
        double normalize_yaw(const double yaw);
    };
}
