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

#include "cpp_mock_drivers/MockDriverNode.h"

/*! \brief The template node for the mock drivers that will handle all ros communication
 *
 * This node will have a constructor that will define which topics it subscribes/published to,
 * as well as what services it will support.
 * 
 * It will also have all default mock driver functionality baked in, e.g. driver discovery and
 * logger services
 */

namespace mock_drivers{

    void MockDriverNode::spin(double rate){
        cnh_.setSpinRate(rate);
        cnh_.spin();
    }

    void MockDriverNode::setSpinCallback(std::function<bool()> cb){
        cnh_.setSpinCallback(cb);
    }

}