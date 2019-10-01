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

#ifndef __BEAM_SEARCH_STRATEGY_HPP__
#define __BEAM_SEARCH_STRATEGY_HPP__

#include "search_strategy.hpp"

namespace arbitrator 
{
    class BeamSearchStrategy : public SearchStrategy
    {
        public:
            BeamSearchStrategy(int beam_width) :
                beam_width_(beam_width) {};
            std::vector<std::pair<cav_msgs::ManeuverPlan, double>> prioritize_plans(std::vector<std::pair<cav_msgs::ManeuverPlan, double>> plans) const;
        private:
            int beam_width_;
    };
}

#endif //__BEAM_SEARCH_STRATEGY_HPP__