#include "platooning_tactical_plugin/platooning_tactical_plugin.h"

namespace platooning_tactical_plugin {

    PlatooningTacticalPlugin::PlatooningTacticalPlugin(ros::NodeHandle &nodeHandle) : nodeHandle_(nodeHandle) {

        if (!readParameters())
        {
            ROS_ERROR("Could not read parameters.");
        }

        waypoint_subscriber_ = nodeHandle_.subscribe("final_waypoint", 1, &PlatooningTacticalPlugin::waypoints_cb, this);
        pose_subscriber_ = nodeHandle_.subscribe("current_pose", 1, &PlatooningTacticalPlugin::pose_cb, this);
        twist_subscriber_ = nodeHandle_.subscribe("current_velocity", 1, &PlatooningTacticalPlugin::twist_cd, this);


        platooning_tactical_plugin_discovery_pub_ = nodeHandle_.advertise<cav_msgs::Plugin>("plugin_discovery", 1);
        plugin_discovery_msg_.name = "PlatooningTacticalPlugin";
        plugin_discovery_msg_.versionId = "v1.0";
        plugin_discovery_msg_.available = true;
        plugin_discovery_msg_.activated = false;
        plugin_discovery_msg_.type = cav_msgs::Plugin::TACTICAL;
        plugin_discovery_msg_.capability = "tactical_plan/plan_trajectory";

        ros::CARMANodeHandle::setSpinCallback([this]() -> bool {
            platooning_tactical_plugin_discovery_pub_.publish(plugin_discovery_msg_);
            return true;
        });

        trajectory_srv_ = nodeHandle_.advertiseService("plugins/PlatooningTachticalPlugin/plan_trajectory", &PlatooningTacticalPlugin::plan_trajectory_cb, this);

        ROS_INFO("Successfully launched node.");
    }

    PlatooningTacticalPlugin::~PlatooningTacticalPlugin()
    {
    }

    bool PlatooningTacticalPlugin::readParameters()
    {
        nodeHandle_.param("trajectory_time_length", trajectory_time_length_, 6.0);
        nodeHandle_.param("trajectory_point_spacing", trajectory_point_spacing_, 0.1);
        return true;
    }

    bool PlatooningTacticalPlugin::plan_trajectory_cb(cav_srvs::PlanTrajectoryRequest &req, cav_srvs::PlanTrajectoryResponse &resp){
        if(req.maneuver_plan.maneuvers[0].lane_following_maneuver.parameters.planning_strategic_plugin == "PlatooningStrategicPlugin") {

            if(req.maneuver_plan.maneuvers[0].lane_following_maneuver.parameters.neogition_type == 2 ){
                if(req.maneuver_plan.maneuvers[0].lane_following_maneuver.start_speed == req.maneuver_plan.maneuvers[0].lane_following_maneuver.end_speed ) {
                    resp.trajectory_plan = trajectory_msg;
                    resp.related_maneuvers.push_back(cav_msgs::Maneuver::LANE_FOLLOWING);
                    resp.maneuver_status.push_back(cav_srvs::PlanTrajectory::Response::MANEUVER_IN_PROGRESS);
                }
            }
            else if(req.maneuver_plan.maneuvers[0].lane_following_maneuver.parameters.neogition_type == 1) {
                resp.trajectory_plan = trajectory_msg;
                resp.related_maneuvers.push_back(cav_msgs::Maneuver::LANE_FOLLOWING);
                resp.maneuver_status.push_back(cav_srvs::PlanTrajectory::Response::MANEUVER_IN_PROGRESS);
            }

        }

        return true;
    }

    void PlatooningTacticalPlugin::waypoints_cb(const autoware_msgs::LaneConstPtr& msg)
    {
        if(msg->waypoints.size() == 0)
        {
            ROS_WARN_STREAM("Received an empty waypoints!");
            return;
        }

        cav_msgs::TrajectoryPlan trajectory;
        trajectory.header.frame_id = msg->header.frame_id;
        trajectory.header.stamp = ros::Time::now();
        trajectory.trajectory_id = boost::uuids::to_string(boost::uuids::random_generator()());
        trajectory.trajectory_points = compose_trajectory_from_waypoints(msg->waypoints);
        waypoints_list = msg->waypoints;
        trajectory_msg = trajectory;

        
        ROS_DEBUG("I heard waypoints");
    };


    std::vector<cav_msgs::TrajectoryPlanPoint> PlatooningTacticalPlugin::compose_trajectory_from_waypoints(std::vector<autoware_msgs::Waypoint> waypoints)
    {
        std::vector<autoware_msgs::Waypoint> partial_waypoints = get_waypoints_in_time_boundary(waypoints, trajectory_time_length_);
        std::vector<cav_msgs::TrajectoryPlanPoint> tmp_trajectory = create_uneven_trajectory_from_waypoints(partial_waypoints);
        std::vector<cav_msgs::TrajectoryPlanPoint> final_trajectory = post_process_traj_points(tmp_trajectory);
        return final_trajectory;
    }

    std::vector<cav_msgs::TrajectoryPlanPoint> PlatooningTacticalPlugin::create_uneven_trajectory_from_waypoints(std::vector<autoware_msgs::Waypoint> waypoints)
    {
        std::vector<cav_msgs::TrajectoryPlanPoint> uneven_traj;
        // TODO land id is not populated because we are not using it in Autoware
        // Adding current vehicle location as the first trajectory point if it is not on the first waypoint
        if(fabs(pose_msg_->pose.position.x - waypoints[0].pose.pose.position.x) > 0.1 || fabs(pose_msg_->pose.position.y - waypoints[0].pose.pose.position.y) > 0.1)
        {
            cav_msgs::TrajectoryPlanPoint starting_point;
            starting_point.target_time = 0.0;
            starting_point.x = pose_msg_->pose.position.x;
            starting_point.y = pose_msg_->pose.position.y;
            uneven_traj.push_back(starting_point);
        }

        double previous_wp_v = waypoints[0].twist.twist.linear.x;
        double previous_wp_x = pose_msg_->pose.position.x;
        double previous_wp_y = pose_msg_->pose.position.y;
        double previous_wp_t = 0.0;
        for(int i = 0; i < waypoints.size(); ++i)
        {
            if(i != 0)
            {
                previous_wp_v = waypoints[i - 1].twist.twist.linear.x;
                previous_wp_x = uneven_traj.back().x;
                previous_wp_y = uneven_traj.back().y;
                previous_wp_t = uneven_traj.back().target_time;
            }
            if(i == 0 && uneven_traj.size() == 0)
            {
                cav_msgs::TrajectoryPlanPoint starting_point;
                starting_point.target_time = 0.0;
                starting_point.x = waypoints[i].pose.pose.position.x;
                starting_point.y = waypoints[i].pose.pose.position.y;
                uneven_traj.push_back(starting_point);
                continue;
            }
            cav_msgs::TrajectoryPlanPoint traj_point;
            // assume the vehicle is starting from stationary state because it is the same assumption made by pure pursuit wrapper node
            double average_speed = previous_wp_v;
            double delta_d = sqrt(pow(waypoints[i].pose.pose.position.x - previous_wp_x, 2) + pow(waypoints[i].pose.pose.position.y - previous_wp_y, 2));
            traj_point.target_time = (delta_d / average_speed) * 1e9 + previous_wp_t;
            traj_point.x = waypoints[i].pose.pose.position.x;
            traj_point.y = waypoints[i].pose.pose.position.y;
            uneven_traj.push_back(traj_point);
        }

        return uneven_traj;
    }

    std::vector<autoware_msgs::Waypoint> PlatooningTacticalPlugin::get_waypoints_in_time_boundary(std::vector<autoware_msgs::Waypoint> waypoints, double time_span)
    {
        std::vector<autoware_msgs::Waypoint> sublist;
        double total_time = 0.0;
        for(int i = 0; i < waypoints.size(); ++i)
        {
            sublist.push_back(waypoints[i]);
            if(i == 0)
            {
                continue;
            }
            double delta_x_square = pow(waypoints[i].pose.pose.position.x - waypoints[i - 1].pose.pose.position.x, 2);
            double delta_y_square = pow(waypoints[i].pose.pose.position.y - waypoints[i - 1].pose.pose.position.y, 2);
            // Here we ignore z attribute because it is not used by Autoware
            
            double delta_d = sqrt(delta_x_square + delta_y_square);
            double average_v = 0.5 * (waypoints[i].twist.twist.linear.x + waypoints[i - 1].twist.twist.linear.x);
            double delta_t = delta_d / average_v;
            total_time += delta_t;
            if(total_time >= time_span)
            {
                break;
            }
        }
        return sublist;
    }

    std::vector<cav_msgs::TrajectoryPlanPoint> PlatooningTacticalPlugin::post_process_traj_points(std::vector<cav_msgs::TrajectoryPlanPoint> trajectory)
    {
        uint64_t current_nsec = ros::Time::now().toNSec();
        for(int i = 0; i < trajectory.size(); ++i)
        {
            trajectory[i].controller_plugin_name = "mpc_follower";
            trajectory[i].planner_plugin_name = "platooning_tactical_plugin";
            trajectory[i].target_time += current_nsec;
        }

        return trajectory;
    }

    void PlatooningTacticalPlugin::pose_cb(const geometry_msgs::PoseStampedConstPtr& msg)
    {
        pose_msg_ = msg;
    }

    void PlatooningTacticalPlugin::twist_cd(const geometry_msgs::TwistStampedConstPtr& msg)
    {
        current_speed_ = msg->twist.linear.x;
    }

}