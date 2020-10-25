#pragma once

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

#include <vector>
#include <cav_msgs/TrajectoryPlan.h>
#include <cav_msgs/Plugin.h>
#include <carma_utils/CARMAUtils.h>
#include <cav_srvs/PlanTrajectory.h>
#include <carma_wm/WMListener.h>
#include <functional>

#include "inlanecruising_plugin.h"
#include "inlanecruising_config.h"

namespace inlanecruising_plugin
{
class InLaneCruisingPluginNode
{
public:
  InLaneCruisingPluginNode(){}

  // general starting point of this node
  void run()
  {
    nh_.reset(new ros::CARMANodeHandle());
    pnh_.reset(new ros::CARMANodeHandle("~"));

    wml_.reset(new carma_wm::WMListener());
    auto wm_ = wml_->getWorldModel();

    inlanecruising_plugin_discovery_pub_ = nh_->advertise<cav_msgs::Plugin>("plugin_discovery", 1);

    InLaneCruisingPluginConfig config;

    pnh_->param<double>("trajectory_time_length", config.trajectory_time_length, config.trajectory_time_length);
    pnh_->param<double>("curve_resample_step_size", config.curve_resample_step_size, config.curve_resample_step_size);
    pnh_->param<int>("downsample_ratio", config.downsample_ratio, config.downsample_ratio);
    pnh_->param<double>("minimum_speed", config.minimum_speed, config.minimum_speed);
    pnh_->param<int>("lookahead_count", config.lookahead_count, config.lookahead_count);
    pnh_->param<int>("moving_average_window_size", config.moving_average_window_size, config.moving_average_window_size);
    pnh_->param<double>("/vehicle_acceleration_limit", config.max_accel, config.max_accel);
    pnh_->param<double>("/vehicle_lateral_accel_limit", config.lateral_accel_limit, config.lateral_accel_limit);
    
    InLaneCruisingPlugin worker(
        wm_, config, std::bind(&InLaneCruisingPluginNode::publishPluginDiscovery, this, std::placeholders::_1));

    trajectory_srv_ = nh_->advertiseService("plugins/InLaneCruisingPlugin/plan_trajectory",
                                            &InLaneCruisingPlugin::plan_trajectory_cb, &worker);

    ros::CARMANodeHandle::setSpinCallback(std::bind(&InLaneCruisingPlugin::onSpin, &worker));
  }

  void publishPluginDiscovery(const cav_msgs::Plugin& msg)
  {
    inlanecruising_plugin_discovery_pub_.publish(msg);
  }

private:
  // node handles
  std::shared_ptr<ros::CARMANodeHandle> nh_, pnh_;
  std::shared_ptr<carma_wm::WMListener> wml_;

  ros::Publisher inlanecruising_plugin_discovery_pub_;

  // ros service servers
  ros::ServiceServer trajectory_srv_;
};

}  // namespace inlanecruising_plugin