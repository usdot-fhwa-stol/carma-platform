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
#include <stop_controlled_intersection_plugin.h>
#include <stop_controlled_intersection_config.h>


namespace stop_controlled_intersection_transit_plugin
{
    /**
 * \brief ROS node for the StopControlledIntersectionTransitPlugin
 */
class StopControlledIntersectionTransitPluginNode
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

    StopControlledIntersectionTacticalPluginConfig config;

    ros::Publisher discovery_pub = nh.advertise<cav_msgs::Plugin>("plugin_discovery", 1);

    // // Determine if we will enable debug publishing by checking the current log level of the node
    // std::map<std::string, ros::console::levels::Level> logger;
    // ros::console::get_loggers(logger);
    // config.publish_debug = logger[ROSCONSOLE_DEFAULT_NAME] == ros::console::levels::Debug;

    StopControlledIntersectionTacticalPlugin worker(wm_, config, [&discovery_pub](auto& msg) { discovery_pub.publish(msg); });
                                            // [trajectory_debug_pub](const auto& msg) { trajectory_debug_pub.publish(msg); });

    ros::ServiceServer trajectory_srv_ = nh.advertiseService("plugins/StopControlledIntersectionTacticalPlugin/plan_trajectory",
                                            &StopControlledIntersectionTacticalPlugin::plan_trajectory_cb, &worker);

    ros::Timer discovery_pub_timer_ = 
      pnh.createTimer(ros::Duration(ros::Rate(10.0)), [&worker](const auto&) { worker.onSpin(); });
      
    ros::CARMANodeHandle::spin();
  }
};

}   //namespace stop_controlled_intersection_plugin