# pragma once
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
#include <boost/shared_ptr.hpp>
#include <carma_utils/CARMAUtils.h>
#include <boost/geometry.hpp>
#include <carma_wm/Geometry.h>
#include <cav_srvs/PlanTrajectory.h>
#include <carma_wm/WMListener.h>
#include <functional>
#include <carma_wm/CARMAWorldModel.h>

#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/geometry/LineString.h>


namespace stopandwait_plugin
{
    using PublishPluginDiscoveryCB = std::function<void(const cav_msgs::Plugin&)>;
    struct PointSpeedPair
    {
        lanelet::BasicPoint2d point;
        double speed=0;
    };

    struct DiscreteCurve
    {
        Eigen::Isometry2d frame;
        std::vector<PointSpeedPair> points;
    };

    class StopandWait
    {
    public:
        StopandWait(){}
    /**
        * \brief Constructor
        * 
        * \param wm Pointer to intialized instance of the carma world model for accessing semantic map data
        * \param plugin_discovery_publisher Callback which will publish the current plugin discovery state
        */
        StopandWait (carma_wm::WorldModelConstPtr wm, PublishPluginDiscoveryCB plugin_discovery_publisher);

        bool plan_trajectory_cb(cav_srvs::PlanTrajectoryRequest& req, cav_srvs::PlanTrajectoryResponse& resp);

        bool onSpin();

        std::vector<PointSpeedPair> maneuvers_to_points(const std::vector<cav_msgs::Maneuver>& maneuvers,
                                                                      double max_starting_downtrack,
                                                                      const carma_wm::WorldModelConstPtr& wm);
        
        std::vector<cav_msgs::TrajectoryPlanPoint> compose_trajectory_from_centerline(
        const std::vector<PointSpeedPair>& points, const cav_msgs::VehicleState& state);

        int getNearestPointIndex(const std::vector<PointSpeedPair>& points, const cav_msgs::VehicleState& state);

        void splitPointSpeedPairs(const std::vector<PointSpeedPair>& points, std::vector<lanelet::BasicPoint2d>* basic_points,
                            std::vector<double>* speeds);

        //std::vector<DiscreteCurve> compute_sub_curves(const std::vector<PointSpeedPair>& basic_points);

        carma_wm::WorldModelConstPtr wm_;
    private:
        
        PublishPluginDiscoveryCB plugin_discovery_publisher_;

        cav_msgs::Plugin plugin_discovery_msg_;
        
    };
}