#pragma once

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
#include <ros/ros.h>
#include <string>
#include <algorithm>
#include <memory>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <lanelet2_core/geometry/Point.h>
#include <trajectory_utils/trajectory_utils.h>
#include <trajectory_utils/conversions/conversions.h>
#include <sstream>
#include <carma_utils/containers/containers.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <Eigen/SVD>
#include <unordered_set>
#include <vector>
#include <cav_msgs/Trajectory.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/geometry/LineString.h>
#include <carma_wm/CARMAWorldModel.h>
#include <carma_utils/containers/containers.h>
#include <carma_wm/Geometry.h>
#include <cav_msgs/TrajectoryPlanPoint.h>
#include <cav_msgs/TrajectoryPlan.h>
#include <math.h>
#include <std_msgs/Float64.h>
#include <cav_msgs/Plugin.h>
#include <carma_utils/CARMAUtils.h>
#include <cav_srvs/PlanTrajectory.h>
#include <carma_wm/WMListener.h>
#include <functional>
#include <autoware_msgs/Lane.h>
#include <carma_debug_msgs/TrajectoryCurvatureSpeeds.h>
#include <light_controlled_intersection_plugin.h>
#include <light_controlled_intersection_config.h>


namespace light_controlled_intersection_transit_plugin
{
    /**
 * \brief ROS node for the LightControlledIntersectionTransitPlugin
 */
class LightControlledIntersectionTransitPluginNode
{
public:
  /**
   * \brief Entrypoint for this node
   */ 
  void run()
  {
    ros::CARMANodeHandle nh;
    ros::CARMANodeHandle pnh("~");
    
    carma_wm::WMListener wml;
    auto wm_ = wml.getWorldModel();

    LightControlledIntersectionTacticalPluginConfig config;

    pnh.param<double>("trajectory_time_length", config.trajectory_time_length, config.trajectory_time_length);
    pnh.param<double>("curve_resample_step_size", config.curve_resample_step_size, config.curve_resample_step_size);
    pnh.param<int>("default_downsample_ratio", config.default_downsample_ratio, config.default_downsample_ratio);
    pnh.param<int>("turn_downsample_ratio", config.turn_downsample_ratio, config.turn_downsample_ratio);
    pnh.param<double>("vehicle_decel_limit_multiplier", config.vehicle_decel_limit_multiplier, config.vehicle_decel_limit_multiplier);
    pnh.param<double>("vehicle_accel_limit_multiplier", config.vehicle_accel_limit_multiplier, config.vehicle_accel_limit_multiplier);
    pnh.param<double>("lat_accel_multiplier", config.lat_accel_multiplier, config.lat_accel_multiplier);
    pnh.param<double>("back_distance", config.back_distance, config.back_distance);
    pnh.param<int>("speed_moving_average_window_size", config.speed_moving_average_window_size,
                     config.speed_moving_average_window_size);
    pnh.param<int>("curvature_moving_average_window_size", config.curvature_moving_average_window_size,
                     config.curvature_moving_average_window_size);
    pnh.param<double>("buffer_ending_downtrack", config.buffer_ending_downtrack, config.buffer_ending_downtrack);
    pnh.param<double>("/vehicle_deceleration_limit", config.vehicle_decel_limit, config.vehicle_decel_limit);
    pnh.param<double>("/vehicle_acceleration_limit", config.vehicle_accel_limit, config.vehicle_accel_limit);
    pnh.param<double>("/vehicle_lateral_accel_limit", config.lateral_accel_limit, config.lateral_accel_limit);
    pnh.param<double>("stop_line_buffer", config.stop_line_buffer, config.stop_line_buffer);
    pnh.param<double>("algorithm_evaluation_distance", config.algorithm_evaluation_distance, config.algorithm_evaluation_distance);
    pnh.param<double>("algorithm_evaluation_period", config.algorithm_evaluation_period, config.algorithm_evaluation_period);
    pnh.param<double>("minimum_speed", config.minimum_speed, config.minimum_speed);
    
    ROS_INFO_STREAM("LightControlledIntersectionTacticalPlugin Params" << config);
    
    config.lateral_accel_limit = config.lateral_accel_limit * config.lat_accel_multiplier;
    config.vehicle_accel_limit = config.vehicle_accel_limit *  config.vehicle_accel_limit_multiplier;
    config.vehicle_decel_limit = config.vehicle_decel_limit *  config.vehicle_decel_limit_multiplier;

    ros::Publisher discovery_pub = nh.advertise<cav_msgs::Plugin>("plugin_discovery", 1);

    LightControlledIntersectionTacticalPlugin worker(wm_, config, [&discovery_pub](auto& msg) { discovery_pub.publish(msg); });

    ros::ServiceServer trajectory_srv_ = nh.advertiseService("plugins/LightControlledIntersectionTacticalPlugin/plan_trajectory",
                                            &LightControlledIntersectionTacticalPlugin::plan_trajectory_cb, &worker);

    ros::Timer discovery_pub_timer_ = 
      pnh.createTimer(ros::Duration(ros::Rate(10.0)), [&worker](const auto&) { worker.onSpin(); });
      
    ros::CARMANodeHandle::spin();
  }
};

}   //namespace light_controlled_intersection_plugin