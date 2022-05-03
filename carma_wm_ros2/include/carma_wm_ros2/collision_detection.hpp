/*
 * Copyright (C) 2022 LEIDOS.
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

#include <carma_perception_msgs/msg/roadway_obstacle_list.hpp>
#include <carma_perception_msgs/msg/roadway_obstacle.hpp>
#include <carma_v2x_msgs/msg/vehicle_size.hpp>
#include <carma_planning_msgs/msg/trajectory_plan.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/foreach.hpp>
#include <vector>
#include <boost/assign/std/vector.hpp>

#include <iostream>
#include <fstream>

#include <carma_wm_ros2/Geometry.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <rclcpp/rclcpp.hpp>

namespace carma_wm {

    namespace collision_detection {

        typedef boost::geometry::model::point<double, 2, boost::geometry::cs::cartesian> point_t;
        typedef boost::geometry::model::polygon<point_t> polygon_t;

        //TODO: verify correctness of descriptions for each of these elements
        struct MovingObject {
            polygon_t                                       object_polygon; //current exterior outline of the object in the x-y plane
            geometry_msgs::msg::Vector3                     linear_velocity;//current velocity vector of the object's centroid, m/s
            std::vector<std::tuple <__uint64_t, polygon_t>> fp;             //zero or more predicted future outline polygons; each
                                                                            // element is (t, polygon), where t is time in future, ms
        };

        /*!
        * Main Function for the CollisionChecking interfacing.
        */

        /*! \brief Main collision detection function to be called when needed to check for collision detection of the vehicle with 
        * the current trajectory plan and the current world objects
        * \param size The size of the host vehicle defined in meters
        * \param rwol The list of Roadway Obstacle
        * \param tp The TrajectoryPlan of the host vehicle
        * \param size The size of the host vehicle defined in meters
        * \param velocity of the host vehicle m/s
        * \return A list of obstacles the provided trajectory plan collides with
        */
        std::vector<carma_perception_msgs::msg::RoadwayObstacle> WorldCollisionDetection(const carma_perception_msgs::msg::RoadwayObstacleList& rwol, 
                                                                    const carma_planning_msgs::msg::TrajectoryPlan& tp, const geometry_msgs::msg::Vector3& size, 
                                                                    const geometry_msgs::msg::Twist& velocity);
        
        /*! \brief Convert RodwayObstable object to the collision_detection::MovingObject 
        * \param rwo A RoadwayObstacle
        */

        collision_detection::MovingObject ConvertRoadwayObstacleToMovingObject(const carma_perception_msgs::msg::RoadwayObstacle& rwo);
        
        /*! \brief Creates collision_detection::MovingObject for the host vehicle using the veloctiy, size, TrajectoryPlan.
        * \param tp The TrajectoryPlan of the host vehicle
        * \param size The size of the host vehicle defined in meters
        * \param veloctiy of the host vehicle
        */

        collision_detection::MovingObject ConvertVehicleToMovingObject(const carma_planning_msgs::msg::TrajectoryPlan& tp, const geometry_msgs::msg::Vector3& size, const geometry_msgs::msg::Twist& veloctiy);                

        /*! \brief .function is to create a monving object with a polygon that represents area that object is going to allocated until a given time
        * by creating a convex hull around the future polygons
        * \param op a MovingObject
        * TODO this function can be optimize due the fact that in large target_time returning the entire volum for path will result in false positive. 
        */

        collision_detection::MovingObject PredictObjectPosition(collision_detection::MovingObject const &op,__uint64_t target_time);
        
        /*! \brief .check Intersection between polygons
        */

        bool CheckPolygonIntersection(collision_detection::MovingObject const &ob_1, collision_detection::MovingObject const &ob_2);
        
        /*! \brief .function is used in WorldCollisionDetection to detection collision between two Moving Object
        */

        bool DetectCollision(collision_detection::MovingObject const &ob_1, collision_detection::MovingObject const &ob_2, __uint64_t target_time);
        
        /*! \brief function to create a polygon representing and object
        */

        template <class P>
        P ObjectToBoostPolygon(const geometry_msgs::msg::Pose& pose, const geometry_msgs::msg::Vector3& size);
    }
}