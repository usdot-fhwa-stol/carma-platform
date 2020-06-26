/*
 * Copyright (C) 2019 LEIDOS.
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

#include <cav_msgs/RoadwayObstacleList.h>
#include <cav_msgs/RoadwayObstacle.h>
#include <cav_msgs/VehicleSize.h>
#include <cav_msgs/TrajectoryPlan.h>
#include <geometry_msgs/Vector3.h>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/foreach.hpp>
#include <vector>
#include <boost/assign/std/vector.hpp>

#include <iostream>
#include <fstream>

#include <carma_wm/Geometry.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


namespace carma_wm {

    namespace collision_detection {

        typedef boost::geometry::model::point<double, 2, boost::geometry::cs::cartesian> point_t;
        typedef boost::geometry::model::polygon<point_t> polygon_t;

        struct MovingObject {
            polygon_t object_polygon;
            geometry_msgs::Vector3 linear_velocity;
            std::vector<polygon_t> future_polygons;
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
        * \param veloctiy of the host vehicle
        * \param target_time amount of unit of time in future to look for collision
        * \return A list of obstacles the provided trajectory plan collides with
        */
        std::vector<cav_msgs::RoadwayObstacle> WorldCollisionDetection(cav_msgs::RoadwayObstacleList& rwol, cav_msgs::TrajectoryPlan& tp, geometry_msgs::Vector3& size, geometry_msgs::Twist& veloctiy, int target_time);
        
        /*! \brief Convert RodwayObstable object to the collision_detection::MovingObject 
            * \param rwo A RoadwayObstacle
        */

        collision_detection::MovingObject ConvertRoadwayObstacleToMovingObject(cav_msgs::RoadwayObstacle& rwo);
        
        /*! \brief Creates collision_detection::MovingObject for the host vehicle using the veloctiy, size, TrajectoryPlan.
        * \param tp The TrajectoryPlan of the host vehicle
        * \param size The size of the host vehicle defined in meters
        * \param veloctiy of the host vehicle
        */

        collision_detection::MovingObject ConvertVehicleToMovingObject(cav_msgs::TrajectoryPlan& tp, geometry_msgs::Vector3& size, geometry_msgs::Twist& veloctiy);                

        /*! \brief .function is to create a monving object with a polygon that represents area that object is going to allocated until a given time
        * by creating a convex hull around the future polygons
        * \param op a MovingObject
        */

        collision_detection::MovingObject PredictObjectPosition(collision_detection::MovingObject op, int target_time);
        
        /*! \brief .check Intersection between polygons
        */

        bool CheckPolygonIntersection(collision_detection::MovingObject ob1, collision_detection::MovingObject ob2);
        
        /*! \brief .function is used in WorldCollisionDetection to detection collision between two Moving Object
        */

        bool DetectCollision(collision_detection::MovingObject ob1, collision_detection::MovingObject ob2, int target_time);
        
        /*! \brief function to create a polygon representing and object
        */

        template <class P>
        P ObjectToBoostPolygon(const geometry_msgs::Pose& pose, const geometry_msgs::Vector3& size);
    }
}