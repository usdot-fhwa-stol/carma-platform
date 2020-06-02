
/*
 * Copyright (C) 2019-2020 LEIDOS.
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
#include <ros/ros.h>
#include <string>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include "unobstructed_lanechange.h"

namespace unobstructed_lanechange
{
    UnobstructedLaneChangePlugin::UnobstructedLaneChangePlugin():
                                    start_speed_(5.0),
                                    target_speed_(6.0),
                                    trajectory_time_length_(6.0),
                                    control_plugin_name_("mpc_follower") {}

    void UnobstructedLaneChangePlugin::initialize()
    {
        nh_.reset(new ros::CARMANodeHandle());
        pnh_.reset(new ros::CARMANodeHandle("~"));
        
        trajectory_srv_ = nh_->advertiseService("plugins/UnobstructedLaneChangePlugin/plan_trajectory", &UnobstructedLaneChangePlugin::plan_trajectory_cb, this);
                
        ubobstructed_lanechange_plugin_discovery_pub_ = nh_->advertise<cav_msgs::Plugin>("plugin_discovery", 1);
        plugin_discovery_msg_.name = "UnobstructedLaneChangePlugin";
        plugin_discovery_msg_.versionId = "v1.0";
        plugin_discovery_msg_.available = true;
        plugin_discovery_msg_.activated = false;
        plugin_discovery_msg_.type = cav_msgs::Plugin::TACTICAL;
        plugin_discovery_msg_.capability = "tactical_plan/plan_trajectory";
        
        pnh_->param<double>("trajectory_time_length", trajectory_time_length_, 6.0);
        pnh_->param<std::string>("control_plugin_name", control_plugin_name_, "NULL");

        ros::CARMANodeHandle::setSpinCallback([this]() -> bool {
            ubobstructed_lanechange_plugin_discovery_pub_.publish(plugin_discovery_msg_);
            return true;
        });

        _wml.reset(new carma_wm::WMListener());
        _wm = _wml->getWorldModel();

    }

    void UnobstructedLaneChangePlugin::run()
    {
    	initialize();
        ros::CARMANodeHandle::setSpinRate(10.0);
        ros::CARMANodeHandle::spin();

    }

    bool UnobstructedLaneChangePlugin::plan_trajectory_cb(cav_srvs::PlanTrajectoryRequest &req, cav_srvs::PlanTrajectoryResponse &resp){
        
        cav_msgs::Maneuver maneuver_msg = req.maneuver_plan.maneuvers[0];
        std::string start_lanelet_id = maneuver_msg.lane_change_maneuver.starting_lane_id;
        double start_downtrack = maneuver_msg.lane_change_maneuver.start_dist;
        start_speed_ = maneuver_msg.lane_change_maneuver.start_speed;
        
        std::string end_lanelet_id = maneuver_msg.lane_change_maneuver.ending_lane_id;
        double end_downtrack = maneuver_msg.lane_change_maneuver.end_dist;
        target_speed_ = maneuver_msg.lane_change_maneuver.end_speed;


        cav_msgs::TrajectoryPlan trajectory;
        
        trajectory.header.stamp = ros::Time::now();
        trajectory.trajectory_id = boost::uuids::to_string(boost::uuids::random_generator()());
        trajectory.trajectory_points = compose_lanechange_trajectory(start_lanelet_id, start_downtrack, end_lanelet_id, end_downtrack);

        resp.trajectory_plan = trajectory;
        resp.related_maneuvers.push_back(cav_msgs::Maneuver::LANE_CHANGE);
        resp.maneuver_status.push_back(cav_srvs::PlanTrajectory::Response::MANEUVER_IN_PROGRESS);

        return true;
    }


    std::vector<cav_msgs::TrajectoryPlanPoint> UnobstructedLaneChangePlugin::compose_lanechange_trajectory(const std::string& start_id, double start_downtrack, const std::string& end_id, double end_downtrack){
        std::vector<double> start_point = extract_point_from_lanelet(start_id, start_downtrack);
        std::vector<double> end_point = extract_point_from_lanelet(end_id, end_downtrack);
        std::vector<cav_msgs::TrajectoryPlanPoint> tmp_trajectory = create_lanechange_trajectory(start_point, end_point);
        std::vector<cav_msgs::TrajectoryPlanPoint> final_trajectory = post_process_traj_points(tmp_trajectory);
        return final_trajectory;
    }
    

    std::vector<cav_msgs::TrajectoryPlanPoint> UnobstructedLaneChangePlugin::create_lanechange_trajectory(std::vector<double> start, std::vector<double> end){
        
        tk::spline spl;
        
        std::vector<cav_msgs::TrajectoryPlanPoint> lc_trajectory;

        // define spline points
        std::vector<double> ptsx;
        ptsx.push_back(start[0]);
        ptsx.push_back((start[0]+end[0])/2);
        ptsx.push_back(end[0]);

        std::vector<double> ptsy;
        ptsy.push_back(start[1]);
        ptsy.push_back((start[1]+end[1])/2);
        ptsy.push_back(end[1]);

        if (ptsx.size()<3 || ptsy.size()<3){
            throw std::invalid_argument("Insufficient Spline Points");
        }

        // set spline points
        spl.set_points(ptsx, ptsy);

        // Calculate how to break up spline points so that we travel at our desired reference velocity
        double target_x = end[0];
        double start_x = start[0];
        double delta_v = (target_speed_ - start_speed_)/num_points;
        double delta_x = (target_x - start_x)/num_points;



        cav_msgs::TrajectoryPlanPoint start_point;
        start_point.x = start[0];
        start_point.y = start[1];
        start_point.target_time = 0.0;
        lc_trajectory.push_back(start_point);

        
        
        
        double prev_x = start[0];
        double prev_y = start[1];
        double prev_t = 0.0;
        double prev_v = start_speed_;

        for (int i=1; i< num_points ; i++){
            double v_point = prev_v + delta_v;
            double x_point = prev_x + delta_x;
            double y_point = spl(x_point);
            double dist = sqrt(pow(x_point-prev_x,2) + pow(y_point-prev_y,2));
            double t_point = dist/v_point + prev_t;

            cav_msgs::TrajectoryPlanPoint traj_point;
        
            traj_point.x = x_point;
            traj_point.y = y_point;
            traj_point.target_time = t_point;

            lc_trajectory.push_back(traj_point);

            prev_x = x_point;
            prev_y = y_point;
            prev_v = v_point;
            prev_t = t_point;

        }

        return lc_trajectory;

    }

    std::vector<cav_msgs::TrajectoryPlanPoint> UnobstructedLaneChangePlugin::post_process_traj_points(std::vector<cav_msgs::TrajectoryPlanPoint> trajectory)
    {
        uint64_t current_nsec = ros::Time::now().toNSec();
        for(int i = 0; i < trajectory.size(); ++i)
        {
            trajectory[i].controller_plugin_name = control_plugin_name_;
            trajectory[i].planner_plugin_name = "unobstructed_lanechange";
            trajectory[i].target_time += current_nsec;
        }

        return trajectory;
    }
    

    std::vector<double> UnobstructedLaneChangePlugin::extract_point_from_lanelet(const std::string& lanelet_id, double downtrack){

        std::vector<double> point;
        auto shortest_path = _wm->getRoute()->shortestPath();
        std::vector<lanelet::ConstLanelet> tmp;
        lanelet::ConstLanelet start_lanelet;

        int lanelet_id_ = std::stoi(lanelet_id);

        if(lanelet_id_ == -1)
        {
            throw std::invalid_argument("Invalid Lanelet ID");
        }

        for (lanelet::ConstLanelet l : shortest_path) {
            if (l.id()==lanelet_id_) start_lanelet = l;
        }

        for (auto centerline_point:start_lanelet.centerline2d()){
            double dt = _wm->routeTrackPos(centerline_point).downtrack;
            if (dt - downtrack <= 1.0){
                double px = centerline_point.x();
                double py = centerline_point.y();
                point.push_back(px);
                point.push_back(py);
            }
        }

        if(point.size() < 2)
        {
            throw std::invalid_argument("Invalid Downtrack Value");
        }

        return point;

    }

}
