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
            // std::vector<geometry_msgs::Pose> future_poses;
            std::vector<polygon_t> future_polygons;
        };

        // template <typename point>
        // class ApplySpeed {
        //     private :
        //         geometry_msgs::Vector3 linear_velocity;
        //         double time;

        //         inline double apply_speed(double distance, double velocity) {
        //             return distance + (velocity * time);
        //         }

        //     public :
        //         ApplySpeed (geometry_msgs::Vector3 lv, double t) {
        //             linear_velocity = lv;
        //             time = t;
        //         }

        //         inline void operator()(point& p)
        //         {
        //             using boost::geometry::get;
        //             using boost::geometry::set;
        //             set<0>(p, apply_speed(get<0>(p), linear_velocity.x));
        //             set<1>(p, apply_speed(get<1>(p), linear_velocity.y));
        //         }
        // };


        /*!
        * Main class for the CollisionChecking interfacing.
        */

        class CollisionChecking {
            public:
                std::vector<cav_msgs::RoadwayObstacle> WorldCollisionDetection(cav_msgs::RoadwayObstacleList rwol, cav_msgs::TrajectoryPlan tp, geometry_msgs::Vector3& size, geometry_msgs::Twist veloctiy, int target_time);
                collision_detection::MovingObject ConvertRoadwayObstacleToMovingObject(cav_msgs::RoadwayObstacle rwo);
                collision_detection::MovingObject ConvertVehicleToMovingObject(cav_msgs::TrajectoryPlan tp, geometry_msgs::Vector3 size, geometry_msgs::Twist veloctiy);                

                collision_detection::MovingObject PredictObjectPosition(collision_detection::MovingObject op, int target_time);
                bool CheckPolygonIntersection(collision_detection::MovingObject ob1, collision_detection::MovingObject ob2);
                bool DetectCollision(collision_detection::MovingObject ob1, collision_detection::MovingObject ob2, int target_time);
                template <class P>
                P ObjectToBoostPolygon(const geometry_msgs::Pose& pose, const geometry_msgs::Vector3& size);

        };
    }
}