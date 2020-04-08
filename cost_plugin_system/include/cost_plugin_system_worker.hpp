/*
 * Copyright (C) 2018-2020 LEIDOS.
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

#pragma once

#include <string>
#include <ros/ros.h>
#include <atomic>
#include <carma_utils/CARMAUtils.h>
#include <cav_srvs/ComputePlanCost.h>

namespace cost_plugin_system
{
class CostPluginSystemWorker
{
public:
    /*!
     * \brief Default constructor for CostPluginSystemWorker
     */
    CostPluginSystemWorker();

    int run();

private:
    // Node handles
    ros::CARMANodeHandle nh_, pnh_;

    // Service servers
    ros::ServiceServer compute_plan_cost_service_server_;
};
} // namespace cost_plugin_system