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


#include "truck_inspection_client.h"

namespace truck_inspection_client
{

    void TruckInspectionClient::initialize()
    {
        nh_.reset(new ros::CARMANodeHandle());
        pnh_.reset(new ros::CARMANodeHandle("~"));
        pnh_->param<std::string>("vin_number", vin_number_);
        
        mo_pub_ = nh_->advertise<cav_msgs::MobilityOperation>("mobility_operation_outbound", 5);
        request_sub_ = nh_->subscribe("mobility_request_inbound", 1, &TruckInspectionClient::requestCallback, this);
        
    }

    void TruckInspectionClient::run()
    {
        initialize();
        ros::CARMANodeHandle::spin();
    }


    void TruckInspectionClient::requestCallback(const cav_msgs::MobilityRequestConstPtr& msg)
    {
        if(msg->strategy == this->INSPECTION_STRATEGY) {
            cav_msgs::MobilityOperation mo_msg;
            mo_msg.strategy = this->INSPECTION_STRATEGY;
            // TODO: need to pack all info we need into this field after truck safety msg is defined.
            mo_msg.strategy_params = this->vin_number_;
            mo_pub_.publish(mo_msg);
        }
    }

}
