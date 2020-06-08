#include "collision_detection.h"

namespace carma_wm {

    namespace collision_detection {

        std::vector<cav_msgs::RoadwayObstacle> CollisionChecking::WorldCollisionDetection(cav_msgs::RoadwayObstacleList rwol, cav_msgs::TrajectoryPlan tp, geometry_msgs::Vector3& size, geometry_msgs::Twist veloctiy, int target_time){

            std::vector<cav_msgs::RoadwayObstacle> rwo_collison;

            collision_detection::MovingObject vehicle_object = CollisionChecking::ConvertVehicleToMovingObject(tp, size, veloctiy);

            for (int i = 0; i <= rwol.roadway_obstacles.size(); i++) {
                collision_detection::MovingObject rwo = CollisionChecking::ConvertRoadwayObstacleToMovingObject(rwol.roadway_obstacles[i]);
                bool collision = CollisionChecking::DetectCollision(vehicle_object, rwo, target_time);
                if(collision) {
                    rwo_collison.push_back(rwol.roadway_obstacles[i]);
                }
            }

            return rwo_collison;
        };

        collision_detection::MovingObject CollisionChecking::ConvertRoadwayObstacleToMovingObject(cav_msgs::RoadwayObstacle rwo){

            collision_detection::MovingObject ro;
            const geometry_msgs::Pose& pose;
            pose.position.x = rwo.object.pose.pose.position.x;
            pose.position.y = rwo.object.pose.pose.position.y;
            geometry_msgs::Vector3& size = rwo.object.size;
            
            ro.object_polygon = carma_wm::geometry::objectToMapPolygon(pose, size);;
            ro.linear_velocity = rwo.object.velocity.twist.linear;

            return ro;
        };

        collision_detection::MovingObject CollisionChecking::ConvertVehicleToMovingObject(cav_msgs::TrajectoryPlan tp, geometry_msgs::Vector3& size, geometry_msgs::Twist veloctiy){
            
            collision_detection::MovingObject v;

            const geometry_msgs::Pose& pose;
            pose.position.x = tp.trajectory_points[0].x;
            pose.position.y = tp.trajectory_points[0].y;

            v.object_polygon = carma_wm::geometry::objectToMapPolygon(pose, size);
            v.linear_velocity = velocity.twist.linear;

            return v;
        };

        bool CollisionChecking::DetectCollision(collision_detection::MovingObject ob_1, collision_detection::MovingObject ob_2, int target_time) {            

            collision_detection::MovingObject ob_1_after = CollisionChecking::PredictObjectPosition(ob_1,target_time);
            collision_detection::MovingObject ob_2_after = CollisionChecking::PredictObjectPosition(ob_2,target_time);

            if(CollisionChecking::CheckPolygonIntersection(ob_1_after, ob_2_after)){
                return true;
            }
            
            return false;
        };

        bool CollisionChecking::CheckPolygonIntersection(collision_detection::MovingObject ob_1, collision_detection::MovingObject ob_2) {            

                std::deque<polygon_t> output;

                boost::geometry::intersection(ob_1.object_polygon, ob_2.object_polygon, output); 

                BOOST_FOREACH(polygon_t const& p, output)
                {
                    if(boost::geometry::area(p) > 0){
                        return true;
                    }
                }

            return false;
        };

        collision_detection::MovingObject CollisionChecking::PredictObjectPosition(collision_detection::MovingObject op, int target_time){

            std::vector<point_t> orignial_polygon_points = op.object_polygon.outer();
            boost::geometry::for_each_point(op.object_polygon, ApplySpeed<point_t>(op.linear_velocity, target_time));

            std::vector<point_t> new_polygon_points = op.object_polygon.outer();
            std::vector<point_t> unioin_polygon_points;

            unioin_polygon_points.reserve( orignial_polygon_points.size() + new_polygon_points.size() ); 
            unioin_polygon_points.insert( unioin_polygon_points.end(), orignial_polygon_points.begin(), orignial_polygon_points.end() );
            unioin_polygon_points.insert( unioin_polygon_points.end(), new_polygon_points.begin(), new_polygon_points.end() );

            polygon_t union_polygon;  
            boost::geometry::assign_points(union_polygon, unioin_polygon_points);

            polygon_t hull_polygon;
            boost::geometry::convex_hull(union_polygon, hull_polygon);

            collision_detection::MovingObject output_object = {hull_polygon, op.linear_velocity};

            return output_object;
        };
    }
}