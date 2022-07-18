#pragma once

/*
 * Copyright (C) 2019-2022 LEIDOS.
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

#include <string>

namespace subsystem_controllers
{
    /**
     * \brief An entry represents a plugins details for the purposes of tracking
     */ 
    struct Entry
    {
        //! Availability flag of a plugin
        bool available_ = false;
        //! Activation flag of a plugin
        bool active_ = false;
        //! Fully specified node name of a plugin
        std::string name_;
        //! Type of the plugin from the message enum in carma_planning_msgs::Plugin
        uint8_t type_ = 0;
        //! The capability string of the plugin
        std::string capability_;
        //! Flag indicating if the user requested this plugin be activated
        bool user_requested_activation_ = false;
        //! Flag indicating if this is a ros1 node
        bool is_ros1_ = false;

        /**
         * \brief All fields constructor
         */ 
        Entry(bool available, bool active, const std::string& name, uint8_t type, const std::string& capability, bool user_requested_activation, bool is_ros1)
            : available_(available), active_(active), name_(name), type_(type), capability_(capability), user_requested_activation_(user_requested_activation), is_ros1_(is_ros1) {}
        

        Entry() = default;
    };
}