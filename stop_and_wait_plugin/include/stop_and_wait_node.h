#pragma once

/*
 * Copyright (C) 2021 LEIDOS.
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
#include "stop_and_wait_plugin.h"
#include "stop_and_wait_config.h"
#include <vector>
#include <cav_msgs/Trajectory.h>
#include <cav_msgs/StopAndWaitManeuver.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/geometry/LineString.h>
#include <carma_wm/CARMAWorldModel.h>
#include <carma_utils/containers/containers.h>
#include <carma_wm/Geometry.h>
#include <cav_msgs/TrajectoryPlanPoint.h>
#include <cav_msgs/TrajectoryPlan.h>
#include <math.h>
#include <std_msgs/Float64.h>

namespace stop_and_wait_plugin
{
class StopandWaitNode
{
public:
  /**
   * \brief General entry point to begin the operation of this class
   */
  void run()
  {
    ros::CARMANodeHandle nh;
    ros::CARMANodeHandle pnh("~");

    carma_wm::WMListener wml;
    wml.setWorldModelUserName("stop_and_wait_plugin");
    auto wm = wml.getWorldModel();

    StopandWaitConfig config;

    pnh.param<double>("minimal_trajectory_duration", config.minimal_trajectory_duration,
                      config.minimal_trajectory_duration);
    pnh.param<double>("stop_timestep", config.stop_timestep, config.stop_timestep);
    pnh.param<double>("trajectory_step_size", config.trajectory_step_size, config.trajectory_step_size);
    pnh.param<double>("accel_limit_multiplier", config.accel_limit_multiplier, config.accel_limit_multiplier);
    pnh.param<double>("/vehicle_acceleration_limit", config.accel_limit, config.accel_limit);
    pnh.param<double>("crawl_speed", config.crawl_speed, config.crawl_speed);
    pnh.param<double>("cernterline_sampling_spacing", config.cernterline_sampling_spacing, config.cernterline_sampling_spacing);
    pnh.param<double>("default_stopping_buffer", config.default_stopping_buffer, config.default_stopping_buffer);

    ros::Publisher plugin_discovery_pub = nh.advertise<cav_msgs::Plugin>("plugin_discovery", 1);

    StopandWait plugin(wm, config, [&plugin_discovery_pub](auto msg) { plugin_discovery_pub.publish(msg); });

    ros::ServiceServer trajectory_srv_ =
        nh.advertiseService("plan_trajectory", &StopandWait::plan_trajectory_cb, &plugin);

    ros::Timer discovery_pub_timer =
        pnh.createTimer(ros::Duration(ros::Rate(10.0)), [&plugin](const auto&) { plugin.spinCallback(); });

    ros::CARMANodeHandle::spin();
  }
};
}  // namespace stop_and_wait_plugin