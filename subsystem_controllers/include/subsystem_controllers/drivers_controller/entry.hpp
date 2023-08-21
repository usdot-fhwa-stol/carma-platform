#pragma once

/*
 * Copyright (C) 2023 LEIDOS.
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
     * \brief An entry represents a driver details for the purposes of tracking
     */ 
    struct Entry
    {
        //! Availability flag of a driver
        bool available_ = false;
        //! Activation flag of a driver
        bool active_ = false;
        //! Fully specified node name of a driver
        std::string name_;
        //! Type of the driver
        uint8_t type_ = 0;
        //! The capability string of the driver
        std::string capability_;
        
        long timestamp_;
        
        //! Flag indicating if this is a ros1 node
        bool is_ros1_ = false;

        /**
         * \brief All fields constructor
         */ 
        Entry(bool available, bool active, const std::string& name, uint8_t type, const std::string& capability, bool is_ros1)
            : available_(available), active_(active), name_(name), type_(type), capability_(capability), is_ros1_(is_ros1) {}
        

        Entry() = default;
    };
}