/*
 * Copyright (C) 2023 LEIDOS.
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

#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <carma_planning_msgs/msg/trajectory_plan.hpp>
#include <carma_planning_msgs/msg/trajectory_plan_point.hpp>
#include <carma_planning_msgs/msg/plugin.hpp>
#include <carma_planning_msgs/msg/maneuver.hpp>
#include <boost/shared_ptr.hpp>
#include <carma_ros2_utils/carma_lifecycle_node.hpp>
#include <boost/geometry.hpp>
#include <carma_planning_msgs/srv/plan_trajectory.hpp>
#include <intersection_transit_maneuvering/call_interface.hpp>


namespace intersection_transit_maneuvering
{
class Servicer: public CallInterface
{
    public:
        /**
         * @brief Construct a new Servicer object
         * 
         */
        Servicer();

        /**
         * @brief Method that uses the ros::Service::call() definition as specified in the ROS API
         * 
         *@param req Incoming PlanTrajectory service request 
         * @param resp Incoming PlanTrajectory service response
         * @return true if successful, otherwise false
         */
        void call(carma_planning_msgs::srv::PlanTrajectory::Request::SharedPtr req, carma_planning_msgs::srv::PlanTrajectory::Response::SharedPtr& resp); 
        
        /**
        * \brief set the trajectory service client
        * 
        * \param client input trajectory service client
        */
        void set_client(carma_ros2_utils::ClientPtr<carma_planning_msgs::srv::PlanTrajectory> srv_client);
    
    private:
        carma_ros2_utils::ClientPtr<carma_planning_msgs::srv::PlanTrajectory> client; 


};

}