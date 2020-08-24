#pragma once

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

#include <ros/ros.h>
#include <carma_utils/CARMAUtils.h>
#include <memory>

namespace port_drayage_plugin
{
    /**
     * Primary Port Drayage Plugin implementation class. Split into this class
     * primarily concerned with the handling of ROS message processing and the
     * PortDrayageWorker class responsible for handling the core business logic
     * of the Port Drayage functionality.
     */ 
    class PortDrayagePlugin
    {
        private:
            std::shared_ptr<ros::CARMANodeHandle> _nh = nullptr;
            std::shared_ptr<ros::CARMANodeHandle> _pnh = nullptr;
            std::shared_ptr<ros::Subscriber> _maneuver_plan_subscriber = nullptr;
            std::shared_ptr<ros::Subscriber> _cur_speed_subscriber = nullptr;
            std::shared_ptr<ros::Publisher> _outbound_mobility_operations_publisher = nullptr;
        public:
            /**
             * \brief Basic constructor for initializing the Port Drayage Plugin
             * 
             * \param nh A shared ptr to a public node handle for this node
             * \param pnh A shared ptr to a private node handle for this node
             */
            PortDrayagePlugin(std::shared_ptr<ros::CARMANodeHandle> nh, 
                std::shared_ptr<ros::CARMANodeHandle> pnh) :
                _nh(nh),
                _pnh(pnh) {};

            /**
             * \brief Testing constructor for initializing the Port Drayage Plugin
             * 
             * Intended for use without ROS, so the nh and pnh are left as null
             */
            PortDrayagePlugin() :
                _nh(nullptr),
                _pnh(nullptr) {};

            /**
             * \brief Begin execution of the Port Drayage Plugin functionality
             * 
             * \return The exit code of the application
             */
            int run();
    };
} // namespace port_drayage_plugin
