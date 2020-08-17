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

#pragma once

#include "cpp_mock_drivers/MockDriver.h"
#include <derived_object_msgs/ObjectWithCovariance.h>
#include <derived_object_msgs/LaneModels.h>

namespace mock_drivers{

    class MockRoadwaySensorDriver : public MockDriver {

        private:

            boost::shared_ptr<ROSComms<derived_object_msgs::ObjectWithCovariance>> object_with_covariance_ptr_;
            boost::shared_ptr<ROSComms<derived_object_msgs::LaneModels>> lane_models_ptr_;

        public:

            MockRoadwaySensorDriver(bool dummy = false);
            int run();
            void parserCB(const cav_msgs::BagData::ConstPtr& msg);
            bool driverDiscovery();

    };

}