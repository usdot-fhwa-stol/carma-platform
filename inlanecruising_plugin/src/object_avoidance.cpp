
#include <inlanecruising_plugin/object_avoidance.h>

namespace inlanecruising_plugin
{
namespace object_avoidance
{
    ObjectAvoidance::ObjectAvoidance(){}
    
    cav_msgs::TrajectoryPlan ObjectAvoidance::update_traj_for_object(cav_msgs::TrajectoryPlan& original_tp, const carma_wm::WorldModelConstPtr& wm_, double current_speed_) 
    {
        
        cav_msgs::TrajectoryPlan update_tpp_vector;

        geometry_msgs::Twist current_velocity;
        current_velocity.linear.x = current_speed_;


        


        // // TODO get roadway object
        std::vector<cav_msgs::RoadwayObstacle> rwol = wm_->getRoadwayObjects();
        cav_msgs::RoadwayObstacleList rwol2;
        rwol2.roadway_obstacles = rwol;

        std::cout << "here 249" << std::endl;
        std::cout << "host_vehicle_size" << host_vehicle_size.x << std::endl;

        std::vector<cav_msgs::RoadwayObstacle> rwol_collision = carma_wm::collision_detection::WorldCollisionDetection(rwol2, original_tp, host_vehicle_size, current_velocity, 10.0);// 10 replace to target_time
        std::cout << "rwol_collision" << rwol_collision.size() << std::endl;
        std::cout << "rwol" << rwol.size() << std::endl;


        std::cout << "here 253" << std::endl;

        // correct the input types
        if(rwol_collision.size() > 0) {

            std::cout << "Collision!" << std::endl;

            // use trajectory utiles to update trajectory plan points
            
            // lead vehicle trjactory
            double x_lead = rwol_collision[0].object.pose.pose.position.x;
            std::cout << "x_lead:  "<< x_lead << std::endl;
            
            // roadway object position
            double gap_time = (x_lead - x_gap)/current_speed_;
            std::cout << "gap_time:  "<< gap_time << std::endl;
            std::cout << "current_speed_:  "<< current_speed_ << std::endl;

            double collision_time = 0; //\TODO comming from carma_wm collision detection

            double goal_velocity = rwol_collision[0].object.velocity.twist.linear.x;
            std::cout << "goal_vel:  "<< goal_velocity << std::endl;

            double goal_pos = rwol_collision[0].object.pose.pose.position.x - goal_velocity * gap_time;
            std::cout << "goal_pos:  "<< goal_pos << std::endl;

            double initial_time = 0;
            double initial_pos = original_tp.trajectory_points[0].x;

            double initial_accel = 0;
            double goal_accel = 0;

            double delta_v_max = 5;// rwol_collision[0].object.velocity.twist.linear.x - max_trajectory_speed(original_tp.trajectory_points);
            std::cout << "delta_v_max:  "<< delta_v_max << std::endl;
            std::cout << "obj_v:  "<< rwol_collision[0].object.velocity.twist.linear.x << std::endl;
            
            __int64_t t_ref = (original_tp.trajectory_points[original_tp.trajectory_points.size() - 1].target_time.toSec() - original_tp.trajectory_points[0].target_time.toSec());
            std::cout << "t_ref1:  "<< original_tp.trajectory_points[0].target_time.toSec() << std::endl;
            std::cout << "t_ref2:  "<< original_tp.trajectory_points[original_tp.trajectory_points.size() - 1].target_time.toSec() << std::endl;
            std::cout << "t_ref:  "<< t_ref << std::endl;

            double t_ph = 4 * delta_v_max / maximum_deceleration_value;
            std::cout << "t_ph:  "<< t_ph << std::endl;

            double tp = 0;

            if(t_ph > tpmin && t_ref < t_ph){
                tp = t_ph;
            }
            else if(t_ph < tpmin){
                tp = tpmin;
            }
            else {
                tp = t_ref;
            }

            std::cout << "tp:  "<< tp << std::endl;

            std::vector<double> values = quintic_coefficient_calculator::quintic_coefficient_calculator(initial_pos, 
                                                                                                        goal_pos, 
                                                                                                        current_speed_, 
                                                                                                        goal_velocity, 
                                                                                                        initial_accel, 
                                                                                                        goal_accel, 
                                                                                                        initial_time, 
                                                                                                        tp);

            // std::cout << "val0: "<< values[0] << std::endl;
            // std::cout << "val1: "<< values[1] << std::endl;
            // std::cout << "val2: "<< values[2] << std::endl;
            // std::cout << "val3: "<< values[3] << std::endl;
            // std::cout << "val4: "<< values[4] << std::endl;
            // std::cout << "val5: "<< values[5] << std::endl;

            std::vector<cav_msgs::TrajectoryPlanPoint> new_trajectory_points;

            new_trajectory_points.push_back(original_tp.trajectory_points[0]);

            // auto shortest_path = wm_->getRoute()->shortestPath();
            // std::vector<lanelet::ConstLanelet> tmp;
            // lanelet::ConstLanelet start_lanelet;

            std::vector<double> original_traj_downtracks = get_relative_downtracks(original_tp);

            for(int i = 1; i < original_tp.trajectory_points.size() - 1; i++ )
            {            
                double traj_target_time = i * tp / original_tp.trajectory_points.size();
                double dt_dist = polynomial_calc(values, traj_target_time);

                int nearest_index = get_nearest_point_index(original_traj_downtracks, dt_dist);

                cav_msgs::TrajectoryPlanPoint new_tpp = original_tp.trajectory_points[nearest_index];

                // for (lanelet::ConstLanelet l : shortest_path) {

                //     if (l.id()== std::stoi(original_tp.trajectory_points[i].lane_id)) {
                //         start_lanelet = l;
                //     }
                // }

                
                
                // new_tpp.target_time = original_tp.trajectory_points[0].target_time +  ros::Duration(traj_target_time);/// ?????????
                // std::cout << "target time: "<< new_tpp.target_time.toSec() << std::endl;
                //  // target time or duration????????
                // std::cout << "CALC: "<< new_tpp.x << std::endl;
                // new_tpp.lane_id = original_tp.trajectory_points[i].lane_id;
                // new_tpp.controller_plugin_name = original_tp.trajectory_points[i].controller_plugin_name;
                // new_tpp.planner_plugin_name = original_tp.trajectory_points[i].planner_plugin_name;

                // // TODO: unit test this loop, seems suspicious
                // for (auto centerline_point:start_lanelet.centerline2d()) {
                //     double dt = wm_->routeTrackPos(centerline_point).downtrack;
                //     if (dt - new_tpp.x <= min_downtrack){
                //         new_tpp.x = centerline_point.x();
                //         new_tpp.y = centerline_point.y();
                //         break;
                //     }
                // }

                // double x_original = original_tp.trajectory_points[i].x - original_tp.trajectory_points[i - 1].x;
                // double t_original = original_tp.trajectory_points[i].target_time.toSec() - original_tp.trajectory_points[i - 1].target_time.toSec();

                // double x_new = new_trajectory_points[i].x - new_trajectory_points[i - 1].x;
                // double t_new = new_trajectory_points[i].target_time.toSec() - new_trajectory_points[i - 1].target_time.toSec();

                // double v_new = x_new/t_new;
                // double v_original = x_original/t_original;

                // if(v_new > v_original){
                //     values = quintic_coefficient_calculator::quintic_coefficient_calculator(original_tp.trajectory_points[i].x, 
                //                                                                             goal_pos, 
                //                                                                             current_speed_, 
                //                                                                             goal_velocity, 
                //                                                                             initial_accel, 
                //                                                                             goal_accel, 
                //                                                                             initial_time, 
                //                                                                             tp);
                // }

                // new_tpp.x = polynomial_calc(values,new_tpp.target_time);


                new_trajectory_points.push_back(new_tpp);
            }

            update_tpp_vector.header = original_tp.header;

            update_tpp_vector.trajectory_id = original_tp.trajectory_id;

            update_tpp_vector.trajectory_points = new_trajectory_points;

            return update_tpp_vector;
        }

        return original_tp;
    } 

    int ObjectAvoidance::get_nearest_point_index(const std::vector<double>& values, const double& target_value) const
    {
        double min_distance = std::numeric_limits<double>::max();
        int i = 0;
        int best_index = 0;
        for (size_t i = 0; i < values.size(); i++)
        {
            double distance = values[i] - target_value;
            if (distance < min_distance)
            {
            best_index = i;
            min_distance = distance;
            }
            i++;
        }

        return best_index;
    }


    std::vector<double> ObjectAvoidance::get_relative_downtracks(cav_msgs::TrajectoryPlan& trajectory_plan)
    {
        std::vector<double> downtracks;
        downtracks.reserve(trajectory_plan.trajectory_points.size());
        // relative downtrack distance of the fist point is 0.0
        downtracks[0] = 0.0;
        for (size_t i=1; i < trajectory_plan.trajectory_points.size(); i++){
            double dx = trajectory_plan.trajectory_points[i].x - trajectory_plan.trajectory_points[i-1].x;
            double dy = trajectory_plan.trajectory_points[i].y - trajectory_plan.trajectory_points[i-1].y;
            downtracks[i] = downtracks[i-1] + sqrt(dx*dx + dy*dy);
        }
        return downtracks;
    }


    double ObjectAvoidance::polynomial_calc(std::vector<double> coeff, double x)
    {
        double result = 0;

        for (size_t i = 0; i < coeff.size(); i++) {
                        
            double value = coeff[i] * pow(x, (int)(coeff.size() - 1 - i));
            result = result + value;
        }
    
        return result;
    }

    // TODO
    double ObjectAvoidance::max_trajectory_speed(std::vector<cav_msgs::TrajectoryPlanPoint> trajectory_points) 
    {
        double max_speed = 0;

        for(int i = 0; i < trajectory_points.size() - 2; i++ )
        {
            double dx = trajectory_points[i + 1].x - trajectory_points[i].x;
            double dy = trajectory_points[i + 1].y - trajectory_points[i].y;
            double d = sqrt(dx*dx + dy*dy); // TODO: replace with similar function from WM
            double t = (trajectory_points[i + 1].target_time.toSec() - trajectory_points[i].target_time.toSec()); ///convert from ms to s
            double v = d/t;

            if(v > max_speed){
                max_speed = v;
            }

        }

        return max_speed;
    }


};  // namespace object_avoidance
};  // namespace inlanecruising_plugin