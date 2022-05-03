
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

#include "carma_wm/collision_detection.h"

namespace carma_wm {

    namespace collision_detection {

        //TODO: consider rewriting this method to improve efficiency; it has object_count*prediction_count*trajectory_point sqrt calls!

        std::vector<cav_msgs::RoadwayObstacle> WorldCollisionDetection(const cav_msgs::RoadwayObstacleList& rwol, const cav_msgs::TrajectoryPlan& tp, 
                                                                        const geometry_msgs::Vector3& size, const geometry_msgs::Twist& velocity) {

            std::vector<cav_msgs::RoadwayObstacle> rwo_collison;

            // collision_detection::MovingObject vehicle_object = ConvertVehicleToMovingObject(tp, size, velocity);

            // lanelet::BasicPoint2d point(original_tp[0].x,original_tp[0].y);

            // double vehicle_downtrack = CARMAWorldModel::routeTrackPos(point).downtrack;

            for (auto i : rwol.roadway_obstacles) {

                // ROS_DEBUG_STREAM("downtrack: ");
                // ROS_DEBUG_STREAM(i.down_track - vehicle_downtrack);

                for (auto j : i.object.predictions) {

                    std::cout << "next trajectory ....." << std::endl << std::endl << std::endl << std::endl << std::endl << std::endl << std::endl;


                    for(size_t k=0; k < tp.trajectory_points.size(); k++) {

                        ROS_DEBUG_STREAM("in for loop");

                        double distancex = (tp.trajectory_points[k].x - j.predicted_position.position.x)*(tp.trajectory_points[k].x - j.predicted_position.position.x);
                        double distancey = (tp.trajectory_points[k].y - j.predicted_position.position.y)*(tp.trajectory_points[k].y - j.predicted_position.position.y);

                        ROS_DEBUG_STREAM("tp.trajectory_points[k].x");
                        ROS_DEBUG_STREAM(tp.trajectory_points[k].x);

                        ROS_DEBUG_STREAM("j.predicted_position.position.x");
                        ROS_DEBUG_STREAM(j.predicted_position.position.x);

                        ROS_DEBUG_STREAM("tp.trajectory_points[k].y");
                        ROS_DEBUG_STREAM(tp.trajectory_points[k].y);

                        ROS_DEBUG_STREAM("j.predicted_position.position.y");
                        ROS_DEBUG_STREAM(j.predicted_position.position.y);

                        std::cout << "distancex" << std::endl;
                        std::cout << tp.trajectory_points[k].x - j.predicted_position.position.x << std::endl;


                        std::cout << "distancey" << std::endl;
                        std::cout << tp.trajectory_points[k].y - j.predicted_position.position.y << std::endl;


                        double calcdistance = sqrt(abs(distancex + distancey));

                        std::cout <<  "calcdistance"<< std::endl;
                        std::cout << calcdistance << std::endl;

                        std::cout <<  "j.header.stamp"<< std::endl;
                        std::cout <<  j.header.stamp << std::endl;

                        std::cout << "tp.trajectory_points[k].target_time"<< std::endl;
                        std::cout << tp.trajectory_points[k].target_time << std::endl;

                        ros::Duration diff= j.header.stamp - tp.trajectory_points[k].target_time;

                        double timediff = diff.toSec();

                        std::cout << "time diff" << std::endl;
                        std::cout << timediff << std::endl;

                        double x = (i.object.size.x - size.x)*(i.object.size.x - size.x);
                        double y = (i.object.size.y - size.y)*(i.object.size.y - size.y);

                        std::cout << "size diff" << std::endl;
                        std::cout << sqrt(x - y) << std::endl;

                        std::cout << "velocity.linear.x" << velocity.linear.x << std::endl;
                        std::cout << "velocity.linear.y" << velocity.linear.y << std::endl;
                        std::cout << "j.predicted_velocity.linear.x" << j.predicted_velocity.linear.x << std::endl;
                        std::cout << "j.predicted_velocity.linear.y" << j.predicted_velocity.linear.y << std::endl;
                        
                        double car_t_x = tp.trajectory_points[k].x - tp.trajectory_points[0].x / velocity.linear.x;
                        // double car_t_y = tp.trajectory_points[k].y - tp.trajectory_points[0].y/velocity.linear.y;

                        double object_t_x = j.predicted_position.position.x - i.object.predictions[0].predicted_position.position.x / i.object.velocity.twist.linear.x;
                        // double object_t_y = j.predicted_position.position.y - i.object.predictions[0].predicted_position.position.y / j.predicted_velocity.linear.y;

                        std::cout << "car_t_x" << car_t_x << std::endl;
                        std::cout << "object_t_x" << object_t_x << std::endl;

                        std::cout << "diff in time stuff " << car_t_x - object_t_x << std::endl;

                        if(timediff <= 5) {
                            if(calcdistance <= sqrt(x - y) ) {
                                rwo_collison.push_back(i);
                                break;
                            }
                        }
                    }
                }


            //     collision_detection::MovingObject rwo = ConvertRoadwayObstacleToMovingObject(i);

            //     for (int j = 0; j < rwo.fp.size(); j++) {
            //         // std::cout << "helloooooo";

            //         std::deque<polygon_t> output;

            //         polygon_t vehicle = std::get<1>(vehicle_object.fp[j]);
            //         polygon_t object = std::get<1>(rwo.fp[j]);

            //         boost::geometry::correct(object);
            //         boost::geometry::correct(vehicle);

            //         boost::geometry::intersection(object, vehicle, output); 

            //         // std::cout << "object " << boost::geometry::wkt( object) << std::endl;
            //         // std::cout << "car " << boost::geometry::wkt(vehicle) << std::endl;

            //         if(output.size() > 0){

            //             // BOOST_FOREACH(polygon_t const& p, output)
            //             // {
            //             //     std::cout << ": " << boost::geometry::area(p) << std::endl;
            //             // }

            //             // std::cout << "yes" << std::endl;
            //             rwo_collison.push_back(i);
            //             break;
            //         }
            //     }
            }



            return rwo_collison;
        }

        collision_detection::MovingObject ConvertRoadwayObstacleToMovingObject(const cav_msgs::RoadwayObstacle& rwo){

            collision_detection::MovingObject mo;
            
            mo.object_polygon = ObjectToBoostPolygon<polygon_t>(rwo.object.pose.pose, rwo.object.size);

            std::tuple <__uint64_t,polygon_t> current_pose(0 , mo.object_polygon);
            mo.fp.push_back(current_pose);

            // Add future polygons for roadway obstacle
            for (auto i : rwo.object.predictions){
                std::tuple <__uint64_t,polygon_t> future_object(i.header.stamp.toNSec() / 1000000, ObjectToBoostPolygon<polygon_t>(i.predicted_position, rwo.object.size));
                
                mo.fp.push_back(future_object);
            }

            mo.linear_velocity = rwo.object.velocity.twist.linear;

            return mo;
        }

        collision_detection::MovingObject ConvertVehicleToMovingObject(const cav_msgs::TrajectoryPlan& tp, const geometry_msgs::Vector3& size, const geometry_msgs::Twist& velocity){
            
            collision_detection::MovingObject v;

            geometry_msgs::Pose pose;
            pose.position.x = tp.trajectory_points[0].x;
            pose.position.y = tp.trajectory_points[0].y;

            Eigen::Vector2d vehicle_vector = {tp.trajectory_points[1].x - tp.trajectory_points[0].x , tp.trajectory_points[1].y - tp.trajectory_points[0].y};

            Eigen::Vector2d x_axis = {1, 0};

            double yaw = std::acos(vehicle_vector.dot(x_axis)/(vehicle_vector.norm() * x_axis.norm()));

            tf2::Quaternion vehicle_orientation;
            vehicle_orientation.setRPY(0, 0, yaw);

            pose.orientation.x = vehicle_orientation.getX();
            pose.orientation.y = vehicle_orientation.getY();
            pose.orientation.z = vehicle_orientation.getZ();
            pose.orientation.w = vehicle_orientation.getW();

            v.object_polygon = ObjectToBoostPolygon<polygon_t>(pose, size);
            v.linear_velocity = velocity.linear;

            for(size_t i=1; i < tp.trajectory_points.size() - 1; i++){

                vehicle_vector = {tp.trajectory_points[i + 1].x - tp.trajectory_points[i].x , tp.trajectory_points[i+1].y - tp.trajectory_points[i].y};
                yaw = std::acos(vehicle_vector.dot(x_axis)/(vehicle_vector.norm() * x_axis.norm()));

                tf2::Quaternion orientation;
                orientation.setRPY(0, 0, yaw);

                geometry_msgs::Pose trajectory_pose;
                trajectory_pose.position.x = tp.trajectory_points[i].x;
                trajectory_pose.position.y = tp.trajectory_points[i].y;

                trajectory_pose.orientation.x = orientation.getX();
                trajectory_pose.orientation.y = orientation.getY();
                trajectory_pose.orientation.z = orientation.getZ();
                trajectory_pose.orientation.w = orientation.getW();

                std::tuple <__uint64_t,polygon_t> future_object(tp.trajectory_points[i].target_time.toNSec(),ObjectToBoostPolygon<polygon_t>(trajectory_pose, size));

                v.fp.push_back(future_object);
            }

            return v;
        }

        bool DetectCollision(collision_detection::MovingObject const &ob_1, collision_detection::MovingObject const &ob_2, __uint64_t target_time) {            
            
            collision_detection::MovingObject ob_1_after = PredictObjectPosition(ob_1,target_time);

            collision_detection::MovingObject ob_2_after = PredictObjectPosition(ob_2,target_time);

            if(CheckPolygonIntersection(ob_1_after, ob_2_after)){
                return true;
            }
            
            return false;
        }

        bool CheckPolygonIntersection(collision_detection::MovingObject const &ob_1, collision_detection::MovingObject const &ob_2) {    

                std::deque<polygon_t> output;

                boost::geometry::intersection(ob_1.object_polygon, ob_2.object_polygon, output); 

                std::cout << boost::geometry::wkt(ob_1.object_polygon) << std::endl;
                std::cout << boost::geometry::wkt(ob_2.object_polygon) << std::endl;
                // std::cout << boost::geometry::wkt(output) << std::endl;

                if(output.size() > 0){
                    return true;
                }

            return false;
        }

        collision_detection::MovingObject PredictObjectPosition(collision_detection::MovingObject const &op, __uint64_t target_time){
            
            int union_polygon_size = 0;
            for (auto i : op.fp){
                if( std::get<0>(i) <= target_time) {
                    union_polygon_size = union_polygon_size + std::get<1>(i).outer().size();
                }
            }

            std::vector<point_t> union_future_polygon_points;
            union_future_polygon_points.reserve(union_polygon_size);

            for (auto i : op.fp){
                if( std::get<0>(i) <= target_time) {
                    union_future_polygon_points.insert( union_future_polygon_points.end(), std::get<1>(i).outer().begin(), std::get<1>(i).outer().end());
                }
            }

            polygon_t union_polygon;  
            boost::geometry::assign_points(union_polygon, union_future_polygon_points);

            polygon_t hull_polygon;
            boost::geometry::convex_hull(union_polygon, hull_polygon);

            std::vector<std::tuple <__uint64_t, collision_detection::polygon_t>> no_future;
            collision_detection::MovingObject output_object = {hull_polygon, op.linear_velocity, no_future};
            
            return output_object;
        }

        template <class P>
        P ObjectToBoostPolygon(const geometry_msgs::Pose& pose, const geometry_msgs::Vector3& size) {

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
