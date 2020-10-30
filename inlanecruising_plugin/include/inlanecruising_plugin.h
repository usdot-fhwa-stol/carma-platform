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
#include <cav_msgs/TrajectoryPlanPoint.h>
#include <cav_msgs/Plugin.h>
#include <autoware_msgs/Lane.h>
#include <boost/shared_ptr.hpp>
#include <carma_utils/CARMAUtils.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <carma_wm/Geometry.h>
#include <cav_srvs/PlanTrajectory.h>
#include <tf2/LinearMath/Matrix3x3.h> // TODO it should be possible to use tf2_eigen instead which would be more efficient since lanelet2 uses eigen under the hood
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <carma_wm/WMListener.h>
#include <functional>

#include "inlanecruising_config.h"
#include "third_party_library/spline.h"

namespace inlanecruising_plugin
{
    using PublishPluginDiscoveryCB = std::function<void(const cav_msgs::Plugin&)>;
    using PointSpeedPair = std::pair<lanelet::BasicPoint2d, double>;

    class InLaneCruisingPlugin
    {

    public:

        InLaneCruisingPlugin(carma_wm::WorldModelConstPtr wm_, InLaneCruisingPluginConfig config, PublishPluginDiscoveryCB plugin_discovery_publisher);

        // service callbacks for carma trajectory planning
        bool plan_trajectory_cb(cav_srvs::PlanTrajectoryRequest &req, cav_srvs::PlanTrajectoryResponse &resp);

        bool onSpin();

        
    private:

        carma_wm::WorldModelConstPtr wm_;
        InLaneCruisingPluginConfig config_;
        PublishPluginDiscoveryCB plugin_discovery_publisher_;

        cav_msgs::Plugin plugin_discovery_msg_;

        // convert waypoints to a trajectory
        std::vector<cav_msgs::TrajectoryPlanPoint> compose_trajectory_from_centerline(const std::vector<PointSpeedPair>& points, const cav_msgs::VehicleState& state);

        // create uneven trajectory from waypoints
        std::vector<cav_msgs::TrajectoryPlanPoint> create_uneven_trajectory_from_points(const std::vector<lanelet::BasicPoint2d>& points,
            const std::vector<double>& speeds, const std::vector<tf2::Quaternion>& orientations, const cav_msgs::VehicleState& state);


    };

}

