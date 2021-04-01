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
#include "stop_and_wait_plugin_config.h"
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
    ros::CARMANodeHandle nh_;
    ros::CARMANodeHandle pnh_("~");

    carma_wm::WMListener wml;
    auto wm = wml.getWorldModel();

    StopandWaitConfig config;

    pnh_->param<double>("minimal_trajectory_duration", config.minimal_trajectory_duration,
                        config.minimal_trajectory_duration);
    pnh_->param<double>("stop_timestep", config.stop_timestep, config.stop_timestep);
    pnh_->param<int>("downsample_ratio", config.downsample_ratio, config.downsample_ratio);
    pnh_->param<double>("/guidance/destination_downtrack_range", config.destination_downtrack_range,
                        config.destination_downtrack_range);

    ros::Publisher plugin_discovery_pub_ = nh_->advertise<cav_msgs::Plugin>("plugin_discovery", 1);

    StopandWait plugin(wm, config, [&plugin_discovery_pub_](auto msg) { plugin_discovery_pub_.publish(msg); });

    ros::ServiceServer trajectory_srv_ =
        nh_->advertiseService("plan_trajectory", &StopandWait::plan_trajectory_cb, &plugin);

    ros::CARMANodeHandle::setSpinCallback(std::bind(&StopandWait::spinCallback, this));

    double spin_rate = pnh_->param<double>("spin_rate_hz", 10.0);
    ros::CARMANodeHandle::setSpinRate(spin_rate);
    ros::CARMANodeHandle::spin();
  }
}  // stop_and_wait_plugin