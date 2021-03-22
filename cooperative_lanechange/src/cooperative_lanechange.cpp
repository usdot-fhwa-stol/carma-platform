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
#include "cooperative_lanechange.h"
#include <unordered_set>
#include <lanelet2_core/geometry/Point.h>
#include <trajectory_utils/trajectory_utils.h>
#include <trajectory_utils/conversions/conversions.h>
#include <carma_utils/containers/containers.h>
#include "smoothing/filters.h"
#include <cav_msgs/MobilityPath.h>
#include <cav_msgs/RoadwayObstacle.h>
#include <lanelet2_extension/traffic_rules/CarmaUSTrafficRules.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <lanelet2_extension/utility/query.h>
#include <lanelet2_extension/utility/utilities.h>

#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/BoundingBox.h>
#include <lanelet2_core/LaneletMap.h>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <cav_msgs/ManeuverPlan.h>

namespace cooperative_lanechange
{
    void CooperativeLaneChangePlugin::initialize()
    {
        nh_.reset(new ros::CARMANodeHandle());
        pnh_.reset(new ros::CARMANodeHandle("~"));
        
        trajectory_srv_ = nh_->advertiseService("plugins/CooperativeLaneChangePlugin/plan_trajectory", &CooperativeLaneChangePlugin::plan_trajectory_cb, this);
                
        cooperative_lanechange_plugin_discovery_pub_ = nh_->advertise<cav_msgs::Plugin>("plugin_discovery", 1);
        plugin_discovery_msg_.name = "CooperativeLaneChangePlugin";
        plugin_discovery_msg_.versionId = "v1.0";
        plugin_discovery_msg_.available = true;
        plugin_discovery_msg_.activated = false;
        plugin_discovery_msg_.type = cav_msgs::Plugin::TACTICAL;
        plugin_discovery_msg_.capability = "tactical_plan/plan_trajectory";
        
        pose_sub_ = nh_->subscribe("current_pose", 1, &CooperativeLaneChangePlugin::pose_cb, this);
        twist_sub_ = nh_->subscribe("current_velocity", 1, &CooperativeLaneChangePlugin::twist_cd, this);
        incoming_mobility_response_ = nh_->subscribe("incoming_mobilty_response", 1 , &CooperativeLaneChangePlugin::mobilityresponse_cb, this);
        
        bsm_sub_ = nh_->subscribe("bsm_outbound", 1, &CooperativeLaneChangePlugin::bsm_cb, this);
        outgoing_mobility_request_ = nh_->advertise<cav_msgs::MobilityRequest>("outgoing_mobility_request", 5); // rate from yield plugin
        lanechange_status_pub_ = nh_->advertise<cav_msgs::LaneChangeStatus>("cooperative_lane_change_status",10);
        //Vehicle params
        pnh_->getParam("vehicle_id",sender_id_);

        //Plugin params
        pnh_->param<double>("trajectory_time_length", trajectory_time_length_, 6.0);
        pnh_->param<std::string>("control_plugin_name", control_plugin_name_, "NULL");
        pnh_->param<double>("minimum_speed", minimum_speed_, 2.2352);
        pnh_->param<double>("max_accel", max_accel_, 1.5);
        pnh_->param<double>("minimum_lookahead_distance", minimum_lookahead_distance_, 5.0);
        pnh_->param<double>("maximum_lookahead_distance", maximum_lookahead_distance_, 25.0);
        pnh_->param<double>("minimum_lookahead_speed", minimum_lookahead_speed_, 2.8);
        pnh_->param<double>("maximum_lookahead_speed", maximum_lookahead_speed_, 13.9);
        pnh_->param<double>("lateral_accel_limit", lateral_accel_limit_, 1.5);
        pnh_->param<double>("moving_average_window_size", moving_average_window_size_, 5);
        pnh_->param<double>("curvature_calc_lookahead_count", curvature_calc_lookahead_count_, 1);
        pnh_->param<int>("downsample_ratio", downsample_ratio_, 8);
        pnh_->param<double>("destination_range",destination_range_, 5);
        pnh_->param<double>("lanechange_time_out",lanechange_time_out_, 6.0);
        //tf listener for looking up earth to map transform 
        tf2_listener_.reset(new tf2_ros::TransformListener(tf2_buffer_));

        wml_.reset(new carma_wm::WMListener());
        // set world model point form wm listener
        wm_ = wml_->getWorldModel();


        ros::CARMANodeHandle::setSpinCallback([this]() -> bool {
            cooperative_lanechange_plugin_discovery_pub_.publish(plugin_discovery_msg_);
            return true;
        });

    }

    void CooperativeLaneChangePlugin::mobilityresponse_cb(const cav_msgs::MobilityResponse &msg){
        cav_msgs::LaneChangeStatus lc_status_msg;
        if(msg.is_accepted){
            is_lanechange_accepted_ = true;
            lc_status_msg.status = cav_msgs::LaneChangeStatus::ACCEPTANCE_RECEIVED;
            lc_status_msg.description = "Received lane merge acceptance";
            
        }
        else{
            is_lanechange_accepted_ = false;
            lc_status_msg.status = cav_msgs::LaneChangeStatus::REJECTION_RECEIVED;
            lc_status_msg.description = "Received lane merge rejection";
        }
        lanechange_status_pub_.publish(lc_status_msg);
    }


    double CooperativeLaneChangePlugin::find_current_gap(int veh2_lanelet_id, double veh2_downtrack){
                
        //find downtrack distance between ego and lag vehicle
        double current_gap = 0.0;
        lanelet::BasicPoint2d ego_pos(pose_msg_.pose.position.x,pose_msg_.pose.position.y);
        double ego_current_downtrack = wm_->routeTrackPos(ego_pos).downtrack;
        
        lanelet::LaneletMapConstPtr const_map(wm_->getMap());
        lanelet::ConstLanelet veh2_lanelet = const_map->laneletLayer.get(veh2_lanelet_id);

        lanelet::BasicPoint2d current_loc(pose_msg_.pose.position.x, pose_msg_.pose.position.y);
        auto current_lanelets = lanelet::geometry::findNearest(const_map->laneletLayer, current_loc, 10);       
        if(current_lanelets.size() == 0)
        {
            ROS_WARN_STREAM("Cannot find any lanelet in map!");
            return true;
        }
        lanelet::ConstLanelet current_lanelet = current_lanelets[0].second;

        //Create temporary route between the two vehicles
        lanelet::ConstLanelet start_lanelet = veh2_lanelet;
        lanelet::ConstLanelet end_lanelet = current_lanelet;
        lanelet::traffic_rules::TrafficRulesUPtr traffic_rules = lanelet::traffic_rules::TrafficRulesFactory::create(lanelet::Locations::Germany,lanelet::Participants::VehicleCar);
        lanelet::routing::RoutingGraphUPtr map_graph = lanelet::routing::RoutingGraph::build(*(wm_->getMap()), *traffic_rules);

        auto temp_route = map_graph->getRoute(start_lanelet, end_lanelet);
        
        auto shortest_path2 = temp_route.get().shortestPath();

        //To find downtrack- distance of veh2 from current to start of end lanelet + downtrack of veh1 in its own current lanelet
        lanelet::BasicPoint2d p = shortest_path2.back().centerline2d().front(); //Start point of last lanelet in path
        
        double downtrack_1 = wm_->routeTrackPos(p).downtrack;
        double remaining_downtrack = downtrack_1 - veh2_downtrack + ego_current_downtrack;
        //double downtrack_1 = map_graph->routeTrackPos(p).downtrack;
        current_gap = remaining_downtrack;


        return current_gap;

    }

    void CooperativeLaneChangePlugin::pose_cb(const geometry_msgs::PoseStampedConstPtr& msg)
    {
        pose_msg_ = geometry_msgs::PoseStamped(*msg.get());
    }
    void CooperativeLaneChangePlugin::twist_cd(const geometry_msgs::TwistStampedConstPtr& msg)
    {
        current_speed_ = msg->twist.linear.x;
    }
    
    void CooperativeLaneChangePlugin::bsm_cb(const cav_msgs::BSMConstPtr& msg)
    {
        bsm_core_ = msg->core_data;
        
    }

    void CooperativeLaneChangePlugin::run()
    {
    	initialize();
        ros::CARMANodeHandle::setSpinRate(10.0);
        ros::CARMANodeHandle::spin();

    }

    

    bool CooperativeLaneChangePlugin::plan_trajectory_cb(cav_srvs::PlanTrajectoryRequest &req, cav_srvs::PlanTrajectoryResponse &resp){
        
        //Check if Lane Change is needed
        std::vector<cav_msgs::Maneuver> maneuver_plan;
        for(const auto maneuver : req.maneuver_plan.maneuvers)
        {
            if(maneuver.type == cav_msgs::Maneuver::LANE_CHANGE)
            {
                maneuver_plan.push_back(maneuver);
            }
        }

        //Current only checking for first lane change maneuver message
        int target_lanelet_id = stoi(maneuver_plan[0].lane_change_maneuver.ending_lane_id);
        double target_downtrack = maneuver_plan[0].lane_change_maneuver.end_dist;
        //get subject vehicle info
        lanelet::BasicPoint2d veh_pos(req.vehicle_state.X_pos_global, req.vehicle_state.Y_pos_global);
        double current_downtrack = wm_->routeTrackPos(veh_pos).downtrack;
        auto current_lanelets = lanelet::geometry::findNearest(wm_->getMap()->laneletLayer, veh_pos, 10);       
        int current_lanelet_id = current_lanelets[0].second.id();
        if(current_lanelet_id == target_lanelet_id && current_downtrack >= target_downtrack - destination_range_){
            cav_msgs::LaneChangeStatus lc_status_msg;
            lc_status_msg.status = cav_msgs::LaneChangeStatus::PLANNING_SUCCESS;
            //No description as per UI documentation
            lanechange_status_pub_.publish(lc_status_msg);
        }

        std::vector<cav_msgs::RoadwayObstacle> rwol = wm_->getRoadwayObjects();
        //Assuming only one object in list 
        int veh2_lanelet_id = rwol[0].lanelet_id;
        double veh2_downtrack = rwol[0].down_track;  //Returns downtrack
        double veh2_speed = rwol[0].object.velocity.twist.linear.x;

        //get current_gap
        double current_gap = find_current_gap(veh2_lanelet_id,veh2_downtrack);
        //get desired gap - desired time gap (default 3s)* relative velocity
        double relative_velocity = current_speed_ - veh2_speed;
        double desired_gap = desired_time_gap_ * relative_velocity;      
         
        if(current_gap < desired_gap){
            return true;
        }
        //plan lanechange without filling in response
        std::vector<cav_msgs::TrajectoryPlanPoint> planned_trajectory_points = plan_lanechange(req);

        //send mobility request
        //Planning for first lane change maneuver
        cav_msgs::MobilityRequest request = create_mobility_request(planned_trajectory_points, maneuver_plan[0]);
        outgoing_mobility_request_.publish(request);
        ros::Time request_sent_time = ros::Time::now();
        cav_msgs::LaneChangeStatus lc_status_msg;
        lc_status_msg.status = cav_msgs::LaneChangeStatus::PLAN_SENT;
        lc_status_msg.description = "Requested lane merge";
        lanechange_status_pub_.publish(lc_status_msg);

        //if ack mobility response, send lanechange response
        if(is_lanechange_accepted_){
            cav_msgs::TrajectoryPlan trajectory_plan;
            trajectory_plan.header.frame_id = "map";
            trajectory_plan.header.stamp = ros::Time::now();
            trajectory_plan.trajectory_id = boost::uuids::to_string(boost::uuids::random_generator()());

            trajectory_plan.trajectory_points = planned_trajectory_points;
            trajectory_plan.initial_longitudinal_velocity = std::max(req.vehicle_state.longitudinal_vel, minimum_speed_);
            resp.trajectory_plan = trajectory_plan;

            for (int i=0; i<req.maneuver_plan.maneuvers.size(); i++){
                if (req.maneuver_plan.maneuvers[i].type == cav_msgs::Maneuver::LANE_CHANGE){
                    resp.related_maneuvers.push_back(i);
                    break;
                }
            }
            resp.maneuver_status.push_back(cav_srvs::PlanTrajectory::Response::MANEUVER_IN_PROGRESS);

        }
        else{
            ros::Time planning_end_time = ros::Time::now();
            ros::Duration passed_time = planning_end_time - request_sent_time;
            if(passed_time.toSec() >= lanechange_time_out_){
                cav_msgs::LaneChangeStatus lc_status_msg;
                lc_status_msg.status = cav_msgs::LaneChangeStatus::TIMED_OUT;
                lc_status_msg.description = "Request timed out for lane merge";
                lanechange_status_pub_.publish(lc_status_msg);
            }
 
        }

        return true;
    }
    
    cav_msgs::MobilityRequest CooperativeLaneChangePlugin::create_mobility_request(std::vector<cav_msgs::TrajectoryPlanPoint>& trajectory_plan, cav_msgs::Maneuver maneuver){

        cav_msgs::MobilityRequest request_msg;
        cav_msgs::MobilityHeader header;
        header.sender_id = sender_id_;
        header.recipient_id = DEFAULT_STRING_;      //To do- find way to query recipient id
        header.sender_bsm_id = bsmIDtoString(bsm_core_);
        header.plan_id = boost::uuids::to_string(boost::uuids::random_generator()());
        header.timestamp = trajectory_plan.front().target_time.toSec();
        request_msg.header = header;

        request_msg.strategy = "carma/cooperative-lane-change";
        request_msg.plan_type.type = cav_msgs::PlanType::CHANGE_LANE_LEFT;
        //Urgency- Currently unassigned

        //Location
        cav_msgs::LocationECEF location;
        location.ecef_x = pose_msg_.pose.position.x;
        location.ecef_y = pose_msg_.pose.position.y;
        location.ecef_z = pose_msg_.pose.position.z;
        //Using trajectory first point time as location timestamp;
        location.timestamp = trajectory_plan.front().target_time.toSec();
        request_msg.location = location;
        //Strategy params
        //Encode JSON with Boost Property Tree
        using boost::property_tree::ptree;
        ptree pt;
        pt.put("speed",maneuver.lane_change_maneuver.start_speed);
        pt.put("start_lanelet",maneuver.lane_change_maneuver.starting_lane_id);
        pt.put("end_lanelet", maneuver.lane_change_maneuver.ending_lane_id);
        std::stringstream body_stream;
        boost::property_tree::json_parser::write_json(body_stream,pt);
        request_msg.strategy_params = body_stream.str();
        //Trajectory
        cav_msgs::Trajectory trajectory;
        //get earth to map tf
        try
        {
            geometry_msgs::TransformStamped tf = tf2_buffer_.lookupTransform("earth", "map", ros::Time(0));
            trajectory = TrajectoryPlantoTrajectory(trajectory_plan, tf);
        }
        catch (tf2::TransformException &ex)
        {
            //Throw exception
            ROS_WARN("%s", ex.what());
        }
        request_msg.trajectory = trajectory;
        request_msg.expiration = trajectory_plan.back().target_time.toSec();
        
        return request_msg;
    }

    cav_msgs::Trajectory CooperativeLaneChangePlugin::TrajectoryPlantoTrajectory(const std::vector<cav_msgs::TrajectoryPlanPoint>& traj_points, const geometry_msgs::TransformStamped& tf) const{
        cav_msgs::Trajectory traj;
        cav_msgs::LocationECEF ecef_location = TrajectoryPointtoECEF(traj_points[0], tf);

        if (traj_points.size()<2){
            ROS_WARN("Received Trajectory Plan is too small");
            traj.offsets = {};
        }
        else{
            for (size_t i=1; i<traj_points.size(); i++){
                
                cav_msgs::LocationOffsetECEF offset;
                cav_msgs::LocationECEF new_point = TrajectoryPointtoECEF(traj_points[i], tf);
                offset.offset_x = new_point.ecef_x - ecef_location.ecef_x;
                offset.offset_y = new_point.ecef_y - ecef_location.ecef_y;
                offset.offset_z = new_point.ecef_z - ecef_location.ecef_z;
                traj.offsets.push_back(offset);
                
            }
        }
        
        
        traj.location = ecef_location;
        return traj;
    }

    cav_msgs::LocationECEF CooperativeLaneChangePlugin::TrajectoryPointtoECEF(const cav_msgs::TrajectoryPlanPoint& traj_point, const geometry_msgs::TransformStamped& tf) const{
        cav_msgs::LocationECEF ecef_point;    

        ecef_point.ecef_x = traj_point.x * tf.transform.translation.x;
        ecef_point.ecef_y = traj_point.y * tf.transform.translation.y;
        ecef_point.ecef_z = 0.0 * tf.transform.translation.z;
       
        return ecef_point;
    } 

    std::vector<cav_msgs::TrajectoryPlanPoint> CooperativeLaneChangePlugin::plan_lanechange(cav_srvs::PlanTrajectoryRequest &req){
        
        lanelet::BasicPoint2d veh_pos(req.vehicle_state.X_pos_global, req.vehicle_state.Y_pos_global);
        double current_downtrack = wm_->routeTrackPos(veh_pos).downtrack;

        //convert maneuver info to route points and speeds
        std::vector<cav_msgs::Maneuver> maneuver_plan;
        for(const auto maneuver : req.maneuver_plan.maneuvers)
        {
            if(maneuver.type == cav_msgs::Maneuver::LANE_CHANGE)
            {
                maneuver_plan.push_back(maneuver);
            }
        }
        auto points_and_target_speeds = maneuvers_to_points(maneuver_plan, current_downtrack, wm_,req.vehicle_state);

        auto downsampled_points = carma_utils::containers::downsample_vector(points_and_target_speeds, downsample_ratio_);

        std::vector<cav_msgs::TrajectoryPlanPoint> trajectory_points;
        trajectory_points = compose_trajectory_from_centerline(downsampled_points, req.vehicle_state);

        return trajectory_points;

    }

    std::vector<PointSpeedPair> CooperativeLaneChangePlugin::maneuvers_to_points(const std::vector<cav_msgs::Maneuver>& maneuvers,
    double max_starting_downtrack, const carma_wm::WorldModelConstPtr& wm,const cav_msgs::VehicleState& state){
        std::vector<PointSpeedPair> points_and_target_speeds;
        std::unordered_set<lanelet::Id> visited_lanelets;

        bool first = true;
        for(const auto& maneuver : maneuvers)
        {
            if(maneuver.type != cav_msgs::Maneuver::LANE_CHANGE)
            {
                throw std::invalid_argument("Cooperative Lane Change does  not support this maneuver type");
            }
            cav_msgs::LaneChangeManeuver lane_change_maneuver = maneuver.lane_change_maneuver;
            
            double starting_downtrack = lane_change_maneuver.start_dist;
            if(first){
                if(starting_downtrack > max_starting_downtrack)
                {
                    starting_downtrack = max_starting_downtrack;
                }
                first = false;
            }
            // //Get lane change route
            double ending_downtrack = lane_change_maneuver.end_dist;
                //Get geometry for maneuver
            if (starting_downtrack >= ending_downtrack)
            {
                throw std::invalid_argument("Start distance is greater than or equal to end distance");
            }

            //get route geometry between starting and ending downtracks
            lanelet::BasicLineString2d route_geometry = create_route_geom(starting_downtrack, stoi(lane_change_maneuver.starting_lane_id), ending_downtrack, wm);
            
            int nearest_pt_index = getNearestRouteIndex(route_geometry,state);
            lanelet::BasicLineString2d future_route_geometry(route_geometry.begin() + nearest_pt_index, route_geometry.end());

            first = true;
            double delta_v = (lane_change_maneuver.end_speed - current_speed_)/future_route_geometry.size();
            double prev_v = current_speed_;
            for(auto p :future_route_geometry)
            {
                if(first && points_and_target_speeds.size() !=0){
                    first = false;
                    continue; // Skip the first point if we have already added points from a previous maneuver to avoid duplicates
                }
                PointSpeedPair pair;
                pair.point = p;
                pair.speed = prev_v+ delta_v;
                points_and_target_speeds.push_back(pair);
                prev_v = pair.speed;

            }
        }

        return points_and_target_speeds;

    }

    std::vector<cav_msgs::TrajectoryPlanPoint> CooperativeLaneChangePlugin::compose_trajectory_from_centerline(
    const std::vector<PointSpeedPair>& points, const cav_msgs::VehicleState& state)
    {
        int nearest_pt_index = getNearestPointIndex(points, state);

        std::vector<PointSpeedPair> future_points(points.begin() + nearest_pt_index + 1, points.end()); // Points in front of current vehicle position
        //auto time_bound_points = constrain_to_time_boundary(future_points, trajectory_time_length_ );

        std::vector<double> final_yaw_values;
        std::vector<double> final_actual_speeds;
        std::vector<lanelet::BasicPoint2d> all_sampling_points;

        std::vector<double> speed_limits;
        std::vector<lanelet::BasicPoint2d> curve_points;
        splitPointSpeedPairs(future_points, &curve_points, &speed_limits);
        std::unique_ptr<smoothing::SplineI> fit_curve = compute_fit(curve_points); // Compute splines based on curve points
        if (!fit_curve)
        {
            throw std::invalid_argument("Could not fit a spline curve along the given trajectory!");
        }


        std::vector<double> distributed_speed_limits;
        distributed_speed_limits.reserve(curve_points.size());

        // compute total length of the trajectory to get correct number of points 
        // we expect using curve_resample_step_size
        std::vector<double> downtracks_raw = carma_wm::geometry::compute_arc_lengths(curve_points);

        int total_step_along_curve= downtracks_raw.size();
        int current_speed_index = 0;
        size_t total_point_size = curve_points.size();

        double step_threshold_for_next_speed = (double)total_step_along_curve / (double)total_point_size;
        double scaled_steps_along_curve = 0.0; // from 0 (start) to 1 (end) for the whole trajectory

        //Geometry already fit to spline for lane change
        for (size_t steps_along_curve = 0; steps_along_curve < total_step_along_curve; steps_along_curve++) // Resample curve at tighter resolution
        {
            
            if ((double)steps_along_curve > step_threshold_for_next_speed)
            {
            step_threshold_for_next_speed += (double)total_step_along_curve / (double) total_point_size;
            current_speed_index ++;
            }
            distributed_speed_limits.push_back(speed_limits[current_speed_index]); // Identify speed limits for resampled points
            
        }


        std::vector<double> yaw_values = carma_wm::geometry::compute_tangent_orientations(curve_points);

        std::vector<double> curvatures = carma_wm::geometry::local_circular_arc_curvatures(
            curve_points, curvature_calc_lookahead_count_);  

        curvatures = smoothing::moving_average_filter(curvatures, moving_average_window_size_);


        std::vector<double> ideal_speeds =
            trajectory_utils::constrained_speeds_for_curvatures(curvatures, lateral_accel_limit_);

        std::vector<double> actual_speeds = apply_speed_limits(ideal_speeds, distributed_speed_limits);

        // Drop last point
        final_yaw_values.insert(final_yaw_values.end(), yaw_values.begin(), yaw_values.end());
        all_sampling_points.insert(all_sampling_points.end(), curve_points.begin(), curve_points.end() );
        final_actual_speeds.insert(final_actual_speeds.end(), actual_speeds.begin(), actual_speeds.end());

        if (all_sampling_points.size() == 0)
        {
            ROS_WARN_STREAM("No trajectory points could be generated");
            return {};
        }

        // Find Lookahead Distance based on Velocity
        double lookahead_distance = get_adaptive_lookahead(state.longitudinal_vel);


        // Apply lookahead speeds
        final_actual_speeds = get_lookahead_speed(all_sampling_points, final_actual_speeds, lookahead_distance);
        
        // Add current vehicle point to front of the trajectory
        lanelet::BasicPoint2d cur_veh_point(state.X_pos_global, state.Y_pos_global);

        all_sampling_points.insert(all_sampling_points.begin(),
                                    cur_veh_point);  // Add current vehicle position to front of sample points

        final_actual_speeds.insert(final_actual_speeds.begin(), std::max(state.longitudinal_vel, minimum_speed_));

        final_yaw_values.insert(final_yaw_values.begin(), state.orientation);

        
        // Compute points to local downtracks
        std::vector<double> downtracks = carma_wm::geometry::compute_arc_lengths(all_sampling_points);

        
        // Apply accel limits
        final_actual_speeds = trajectory_utils::apply_accel_limits_by_distance(downtracks, final_actual_speeds,
                                                                                max_accel_, max_accel_);

        
        final_actual_speeds = smoothing::moving_average_filter(final_actual_speeds, moving_average_window_size_);

        for (auto& s : final_actual_speeds)  // Limit minimum speed. TODO how to handle stopping?
        {
            s = std::max(s, minimum_speed_);
        }

        // Convert speeds to times
        std::vector<double> times;
        trajectory_utils::conversions::speed_to_time(downtracks, final_actual_speeds, &times);

        
        // Build trajectory points
        // TODO When more plugins are implemented that might share trajectory planning the start time will need to be based
        // off the last point in the plan if an earlier plan was provided
        std::vector<cav_msgs::TrajectoryPlanPoint> traj_points =
            trajectory_from_points_times_orientations(all_sampling_points, times, final_yaw_values, ros::Time::now());

        return traj_points;

    }

    std::vector<cav_msgs::TrajectoryPlanPoint> CooperativeLaneChangePlugin::trajectory_from_points_times_orientations(
    const std::vector<lanelet::BasicPoint2d>& points, const std::vector<double>& times, const std::vector<double>& yaws,
    ros::Time startTime)
    {
        if (points.size() != times.size() || points.size() != yaws.size())
        {
            throw std::invalid_argument("All input vectors must have the same size");
        }

        std::vector<cav_msgs::TrajectoryPlanPoint> traj;
        traj.reserve(points.size());

        for (int i = 0; i < points.size(); i++)
        {
            cav_msgs::TrajectoryPlanPoint tpp;
            ros::Duration relative_time(times[i]);
            tpp.target_time = startTime + relative_time;
            tpp.x = points[i].x();
            tpp.y = points[i].y();
            tpp.yaw = yaws[i];

            tpp.controller_plugin_name = "default";
            tpp.planner_plugin_name = plugin_discovery_msg_.name;

            traj.push_back(tpp);
        }

        return traj;
    }

    double CooperativeLaneChangePlugin::get_adaptive_lookahead(double velocity)
    {
        // lookahead:
        // v<10kph:  5m
        // 10kph<v<50kph:  0.5*v
        // v>50kph:  25m

        double lookahead = minimum_lookahead_distance_;

        if (velocity < minimum_lookahead_speed_)
        {
            lookahead = minimum_lookahead_distance_;
        } 
        else if (velocity >= minimum_lookahead_speed_ && velocity < maximum_lookahead_speed_)
        {
            lookahead = 2.0 * velocity;
        } 
        else lookahead = maximum_lookahead_distance_;

        return lookahead;

    }

    std::vector<double> CooperativeLaneChangePlugin::get_lookahead_speed(const std::vector<lanelet::BasicPoint2d>& points, const std::vector<double>& speeds, const double& lookahead)
    {
  
        if (lookahead < minimum_lookahead_distance_)
        {
            throw std::invalid_argument("Invalid lookahead value");
        }

        if (speeds.size() < 1)
        {
            throw std::invalid_argument("Invalid speeds vector");
        }

        if (speeds.size() != points.size())
        {
            throw std::invalid_argument("Speeds and Points lists not same size");
        }

        std::vector<double> out;
        out.reserve(speeds.size());

        for (int i = 0; i < points.size(); i++)
        {
            int idx = i;
            double min_dist = std::numeric_limits<double>::max();
            for (int j=i+1; j < points.size(); j++){
            double dist = lanelet::geometry::distance2d(points[i],points[j]);
            if (abs(lookahead - dist) <= min_dist){
                idx = j;
                min_dist = abs(lookahead - dist);
            }
            }
            out.push_back(speeds[idx]);
        }
        
        return out;
        }

    std::vector<double> CooperativeLaneChangePlugin::apply_speed_limits(const std::vector<double> speeds,
                                                             const std::vector<double> speed_limits)
    {
        if (speeds.size() != speed_limits.size())
        {
            throw std::invalid_argument("Speeds and speed limit lists not same size");
        }
        std::vector<double> out;
        for (size_t i = 0; i < speeds.size(); i++)
        {
            out.push_back(std::min(speeds[i], speed_limits[i]));
        }

        return out;
    }

    std::unique_ptr<smoothing::SplineI>
    CooperativeLaneChangePlugin::compute_fit(const std::vector<lanelet::BasicPoint2d>& basic_points)
    {
        if (basic_points.size() < 3)
        {
            ROS_WARN_STREAM("Insufficient Spline Points");
            return nullptr;
        }
        std::unique_ptr<smoothing::SplineI> spl = std::make_unique<smoothing::BSpline>();
        spl->setPoints(basic_points);
        return spl;
    }


    Eigen::Isometry2d CooperativeLaneChangePlugin::compute_heading_frame(const lanelet::BasicPoint2d& p1,
                                                              const lanelet::BasicPoint2d& p2)
    {
    Eigen::Rotation2Dd yaw(atan2(p2.y() - p1.y(), p2.x() - p1.x()));

    return carma_wm::geometry::build2dEigenTransform(p1, yaw);
    }

    std::vector<PointSpeedPair> CooperativeLaneChangePlugin::constrain_to_time_boundary(const std::vector<PointSpeedPair>& points,double time_span)
    {
        std::vector<lanelet::BasicPoint2d> basic_points;
        std::vector<double> speeds;
        splitPointSpeedPairs(points, &basic_points, &speeds);

        std::vector<double> downtracks = carma_wm::geometry::compute_arc_lengths(basic_points);

        size_t time_boundary_exclusive_index =
        trajectory_utils::time_boundary_index(downtracks, speeds, trajectory_time_length_);

        if (time_boundary_exclusive_index == 0)
        {
            throw std::invalid_argument("No points to fit in timespan"); 
        }

        std::vector<PointSpeedPair> time_bound_points;
        time_bound_points.reserve(time_boundary_exclusive_index);

        if (time_boundary_exclusive_index == points.size())
        {
            time_bound_points.insert(time_bound_points.end(), points.begin(),
                                    points.end());  // All points fit within time boundary
        }
        else
        {
            time_bound_points.insert(time_bound_points.end(), points.begin(),
                                    points.begin() + time_boundary_exclusive_index - 1);  // Limit points by time boundary
        }

        return time_bound_points;

    }
    void CooperativeLaneChangePlugin::splitPointSpeedPairs(const std::vector<PointSpeedPair>& points,
                                                std::vector<lanelet::BasicPoint2d>* basic_points,
                                                std::vector<double>* speeds)
    {
        basic_points->reserve(points.size());
        speeds->reserve(points.size());

        for (const auto& p : points)
        {
            basic_points->push_back(p.point);
            speeds->push_back(p.speed);
        }
    }

    int CooperativeLaneChangePlugin::getNearestRouteIndex(lanelet::BasicLineString2d& points, const cav_msgs::VehicleState& state)
    {
        lanelet::BasicPoint2d veh_point(state.X_pos_global, state.Y_pos_global);
                double min_distance = std::numeric_limits<double>::max();
        int i = 0;
        int best_index = 0;
        for (const auto& p : points)
        {
            double distance = lanelet::geometry::distance2d(p,veh_point);
            if (distance < min_distance)
            {
            best_index = i;
            min_distance = distance;
            }
            i++;
        }
        return best_index;

    }

    int CooperativeLaneChangePlugin::getNearestPointIndex(const std::vector<PointSpeedPair>& points,
                                               const cav_msgs::VehicleState& state)
    {
        lanelet::BasicPoint2d veh_point(state.X_pos_global, state.Y_pos_global);
        ROS_DEBUG_STREAM("veh_point: " << veh_point.x() << ", " << veh_point.y());
        double min_distance = std::numeric_limits<double>::max();
        int i = 0;
        int best_index = 0;
        for (const auto& p : points)
        {
            double distance = lanelet::geometry::distance2d(p.point, veh_point);
            ROS_DEBUG_STREAM("distance: " << distance);
            ROS_DEBUG_STREAM("p: " << p.point.x() << ", " << p.point.y());
            if (distance < min_distance)
            {
            best_index = i;
            min_distance = distance;
            }
            i++;
        }

        return best_index;
    }

    lanelet::BasicLineString2d  CooperativeLaneChangePlugin::create_lanechange_path(lanelet::BasicPoint2d start, lanelet::ConstLanelet& start_lanelet, lanelet::BasicPoint2d end, lanelet::ConstLanelet& end_lanelet)
    {
        std::vector<lanelet::BasicPoint2d> centerline_points={};
        lanelet::BasicLineString2d centerline_start_lane = start_lanelet.centerline2d().basicLineString();
        lanelet::BasicLineString2d centerline_end_lane = end_lanelet.centerline2d().basicLineString();

        lanelet::BasicPoint2d start_lane_pt  = centerline_start_lane[0];
        lanelet::BasicPoint2d end_lane_pt = centerline_end_lane[0];
        double dist = sqrt(pow((end_lane_pt.x() - start_lane_pt.x()),2) + pow((end_lane_pt.y() - start_lane_pt.y()),2));
        int total_points = centerline_start_lane.size();
        double delta_step = 1.0/total_points;

        centerline_points.push_back(start_lane_pt);

        for(int i=1;i<total_points;i++){
            lanelet::BasicPoint2d current_position;
            start_lane_pt = centerline_start_lane[i];
            end_lane_pt = centerline_end_lane[i];
            double delta = delta_step*i;
            current_position.x() = end_lane_pt.x()*delta + (1-delta)* start_lane_pt.x();
            current_position.y() = end_lane_pt.y()*delta + (1-delta)* start_lane_pt.y();

            centerline_points.push_back(current_position);

        }

        std::unique_ptr<smoothing::SplineI> fit_curve = compute_fit(centerline_points);
        if(!fit_curve)
        {
            throw std::invalid_argument("Could not fit a spline curve along the given trajectory!");
        }

        lanelet::BasicLineString2d lc_route;
        //lc_route.push_back(start);
        double scaled_steps_along_curve = 0.0; // from 0 (start) to 1 (end) for the whole trajectory
        for(int i =0;i<centerline_points.size();i++){
            lanelet::BasicPoint2d p = (*fit_curve)(scaled_steps_along_curve);
            lc_route.push_back(p);
            scaled_steps_along_curve += 1.0/total_points;
        }
        lc_route.push_back(end);

        return lc_route;
    }

    lanelet::BasicLineString2d CooperativeLaneChangePlugin::create_route_geom( double starting_downtrack, int starting_lane_id, double ending_downtrack, const carma_wm::WorldModelConstPtr& wm)
    {
        //Create route geometry for lane change maneuver
        //Starting downtrack maybe in the previous lanelet, if it is, have a lane follow till new lanelet starts
        std::vector<lanelet::ConstLanelet> lanelets_in_path = wm->getLaneletsBetween(starting_downtrack, ending_downtrack, true);
        lanelet::BasicLineString2d centerline_points = {};
        int lane_change_iteration = 0;
        if(lanelets_in_path[0].id() != starting_lane_id){
            bool lane_change_req= false;
            //Check if future lanelet is lane change lanelet
            for(int i=0;i<lanelets_in_path.size();i++){
                if(lanelets_in_path[i].id() == starting_lane_id){
                    lane_change_req = true;
                    lane_change_iteration = i;
                    break;
                }
            }
            if(!lane_change_req){
                throw std::invalid_argument("Current path does not require a lane change. Request Incorrectly sent to Cooperative lane change plugin");
            }
            //lane_follow till lane_change req
            lanelet::BasicLineString2d new_points =lanelets_in_path[lane_change_iteration-1].centerline2d().basicLineString();
            centerline_points.insert(centerline_points.end(), new_points.begin(), new_points.end());
        }

            lanelet::BasicLineString2d first=lanelets_in_path[lane_change_iteration].centerline2d().basicLineString();
            lanelet::BasicPoint2d start = first.front();
            lanelet::BasicLineString2d last=lanelets_in_path.back().centerline2d().basicLineString();
            lanelet::BasicPoint2d end = last.back();
            
            lanelet::BasicLineString2d new_points = create_lanechange_path(start,lanelets_in_path[lane_change_iteration], end, lanelets_in_path.back());
            centerline_points.insert(centerline_points.end(),new_points.begin(),new_points.end() );
    
        return centerline_points;
        
    }
}