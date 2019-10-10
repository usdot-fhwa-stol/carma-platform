/*
 * Copyright (C) 2019 LEIDOS.
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

#ifndef __PLUGIN_NEIGHBOR_GENERATOR_HPP__
#define __PLUGIN_NEIGHBOR_GENERATOR_HPP__

#include "neighbor_generator.hpp"
#include "capabilities_interface.hpp"

namespace arbitrator
{
    /**
     * \brief Implementation of the NeighborGenerator interface using plugins
     * 
     * Queries plugins via the CapabilitiesInterface to contribute additional
     * maneuvers as the potential child/neighbor nodes of a plan in progress.
     * 
     * \tparam T The type of CapabilitiesInterface to use. Templated to enable 
     *      testing
     */
    template <class T>
    class PluginNeighborGenerator : public NeighborGenerator
    {
        public:
            /**
             * Constructor for PluginNeighborGenerator
             * \param ci A capabilties interface for accessing the plugins
             */
            PluginNeighborGenerator(T &ci) :
                ci_(ci) {};

            /**
             * \brief Initialize this PluginNeighborGenerator for use
             * 
             * Loads the plugins needed from the CapabilitiesInterface and prepares
             * the service clients
             */
            void initalize();
            std::vector<cav_msgs::ManeuverPlan> generate_neighbors(cav_msgs::ManeuverPlan plan);
        private:
            T &ci_;
    };
};

#include "plugin_neighbor_generator.tpp"

#endif //__PLUGIN_NEIGHBOR_GENERATOR_HPP__