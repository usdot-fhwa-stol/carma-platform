#include "carma_wm/collision_detection.h"

namespace carma_wm {

    namespace collision_detection {

        std::vector<cav_msgs::RoadwayObstacle> CollisionChecking::WorldCollisionDetection(cav_msgs::RoadwayObstacleList rwol, cav_msgs::TrajectoryPlan tp, geometry_msgs::Vector3& size, geometry_msgs::Twist veloctiy, int target_time){

            std::cout << "start function" << std::endl;

            std::vector<cav_msgs::RoadwayObstacle> rwo_collison;

            collision_detection::MovingObject vehicle_object = CollisionChecking::ConvertVehicleToMovingObject(tp, size, veloctiy);

            std::cout << "after ConvertVehicleToMovingObjects" << rwol.roadway_obstacles.size() << std::endl;

            for (int i = 0; i < rwol.roadway_obstacles.size(); i++) {

                std::cout << "start ConvertRoadwayObstacleToMovingObject" << std::endl;

                collision_detection::MovingObject rwo = CollisionChecking::ConvertRoadwayObstacleToMovingObject(rwol.roadway_obstacles[i]);
                std::cout << "after ConvertRoadwayObstacleToMovingObject" << std::endl;

                bool collision = CollisionChecking::DetectCollision(vehicle_object, rwo, target_time);
                std::cout << "after DetectCollision" << std::endl;

                if(collision) {
                    std::cout << "after rwo_collison" << std::endl;
                    rwo_collison.push_back(rwol.roadway_obstacles[i]);
                }
            }

            return rwo_collison;
        };

        collision_detection::MovingObject CollisionChecking::ConvertRoadwayObstacleToMovingObject(cav_msgs::RoadwayObstacle rwo){

            collision_detection::MovingObject ro;
            
            ro.object_polygon = CollisionChecking::ObjectToBoostPolygon<polygon_t>(rwo.object.pose.pose, rwo.object.size);

            for (auto i : rwo.object.predictions){
                ro.future_polygons.push_back(CollisionChecking::ObjectToBoostPolygon<polygon_t>(i.predicted_position, rwo.object.size));
                // ro.future_poses.push_back(i.predicted_position);
            }

            ro.linear_velocity = rwo.object.velocity.twist.linear;

            return ro;
        };

        collision_detection::MovingObject CollisionChecking::ConvertVehicleToMovingObject(cav_msgs::TrajectoryPlan tp, geometry_msgs::Vector3 size, geometry_msgs::Twist veloctiy){
            
            collision_detection::MovingObject v;

            geometry_msgs::Pose pose;
            pose.position.x = tp.trajectory_points[0].x;
            pose.position.y = tp.trajectory_points[0].y;

            tf2::Quaternion tf_orientation;
            tf_orientation.setRPY(0, 0, 1.5708);

            pose.orientation.x = tf_orientation.getX();
            pose.orientation.y = tf_orientation.getY();
            pose.orientation.z = tf_orientation.getZ();
            pose.orientation.w = tf_orientation.getW();


            v.object_polygon = CollisionChecking::ObjectToBoostPolygon<polygon_t>(pose, size);
            v.linear_velocity = veloctiy.linear;

            for (auto i : tp.trajectory_points){
                geometry_msgs::Pose pose;
                pose.position.x = i.x;
                pose.position.y = i.y;
                // v.future_poses.push_back(pose);
                v.future_polygons.push_back(CollisionChecking::ObjectToBoostPolygon<polygon_t>(pose, size));
            }

            return v;
        };

        bool CollisionChecking::DetectCollision(collision_detection::MovingObject ob_1, collision_detection::MovingObject ob_2, int target_time) {            
            std::cout << "DetectCollision" << boost::geometry::wkt(ob_1.object_polygon) << std::endl;
            std::cout << "DetectCollision" << boost::geometry::wkt(ob_2.object_polygon) << std::endl;

            collision_detection::MovingObject ob_1_after = CollisionChecking::PredictObjectPosition(ob_1,target_time);
            collision_detection::MovingObject ob_2_after = CollisionChecking::PredictObjectPosition(ob_2,target_time);

            if(CollisionChecking::CheckPolygonIntersection(ob_1_after, ob_2_after)){
                return true;
            }
            
            return false;
        };

        bool CollisionChecking::CheckPolygonIntersection(collision_detection::MovingObject ob_1, collision_detection::MovingObject ob_2) {    



                std::deque<polygon_t> output;
                std::cout << "CheckPolygonIntersection" << boost::geometry::wkt(ob_1.object_polygon) << std::endl;
                std::cout << "CheckPolygonIntersection" << boost::geometry::wkt(ob_2.object_polygon) << std::endl;

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

            int size = 0;
            for(int i = 0; i< target_time; i++){
                size = size + op.future_polygons[i].outer().size();
            }

            std::vector<point_t> unioin_polygon_points;

            unioin_polygon_points.reserve(size); 

            for(int i = 0; i< target_time; i++){
                unioin_polygon_points.insert( unioin_polygon_points.end(), op.future_polygons[i].outer().begin(), op.future_polygons[i].outer().end());
            }

            polygon_t union_polygon;  
            boost::geometry::assign_points(union_polygon, unioin_polygon_points);

            polygon_t hull_polygon;
            boost::geometry::convex_hull(union_polygon, hull_polygon);

            collision_detection::MovingObject output_object = {hull_polygon, op.linear_velocity};
            
            
            std::cout << "PredictObjectPosition" << boost::geometry::wkt(op.future_polygons[0]) << std::endl;

            return output_object;
        };

        template <class P>
        P CollisionChecking::ObjectToBoostPolygon(const geometry_msgs::Pose& pose, const geometry_msgs::Vector3& size) {

            tf2::Transform object_tf;
            tf2::fromMsg(pose, object_tf);

            double half_x_bound = size.x / 2;
            double half_y_bound = size.y / 2;

            // 4 corners of the object starting with upper left and moving in clockwise direction in pose frame
            tf2::Vector3 obj_p1(half_x_bound, half_y_bound, 0);
            tf2::Vector3 obj_p2(half_x_bound, -half_y_bound, 0);
            tf2::Vector3 obj_p3(-half_x_bound, -half_y_bound, 0);
            tf2::Vector3 obj_p4(-half_x_bound, half_y_bound, 0);

            tf2::Vector3 obj_p1_map = object_tf * obj_p1;
            tf2::Vector3 obj_p2_map = object_tf * obj_p2;
            tf2::Vector3 obj_p3_map = object_tf * obj_p3;
            tf2::Vector3 obj_p4_map = object_tf * obj_p4;

            point_t p1(obj_p1_map.getX(), obj_p1_map.getY());
            point_t p2(obj_p2_map.getX(), obj_p2_map.getY());
            point_t p3(obj_p3_map.getX(), obj_p3_map.getY());
            point_t p4(obj_p4_map.getX(), obj_p4_map.getY());


            P p;
            p.outer().push_back(p1);
            p.outer().push_back(p2);
            p.outer().push_back(p3);
            p.outer().push_back(p4);

            return p;
        }

    }
}