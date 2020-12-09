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

#include <cav_msgs/Plugin.h>
#include <carma_utils/CARMAUtils.h>
#include <cav_srvs/PlanTrajectory.h>
#include <carma_wm/WMListener.h>
#include <functional>

#include "stopandwait.h"

namespace stopandwait_plugin
{
    class StopandWaitNode
    {
        public:
        void run()
        {
            ros::CARMANodeHandle nh;
            ros::CARMANodeHandle pnh("~");

            carma_wm::WMListener wml;
            auto wm_ = wml.getWorldModel();

            ros::Publisher discovery_pub = nh.advertise<cav_msgs::Plugin>("plugin_discovery",1);

            
            //ros::CARMANodeHandle::setSpinCallback(std::bind(&StopandWait::onSpin, &worker));
        }
    };
}