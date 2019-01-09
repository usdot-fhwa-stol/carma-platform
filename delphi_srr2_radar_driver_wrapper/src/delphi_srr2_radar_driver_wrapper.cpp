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

#include "delphi_srr2_radar_driver_wrapper.h"

DelphiSrr2RadarDriverWrapper::DelphiSrr2RadarDriverWrapper(int argc, char **argv, const std::string &name) : DriverWrapper (argc, argv, name) {}

DelphiSrr2RadarDriverWrapper::~DelphiSrr2RadarDriverWrapper() {}

void DelphiSrr2RadarDriverWrapper::initialize() {
    srr_status5_sub_ = nh_->subscribe("srr_status5", 10, &DelphiSrr2RadarDriverWrapper::srr_status5_cb, this);
    status_.sensor = true;
}

void DelphiSrr2RadarDriverWrapper::pre_spin() {}

void DelphiSrr2RadarDriverWrapper::post_spin() {}

void DelphiSrr2RadarDriverWrapper::shutdown() {}

void DelphiSrr2RadarDriverWrapper::srr_status5_cb(const delphi_srr_msgs::SrrStatus5ConstPtr &msg)
{
    switch(msg->CAN_TX_SYSTEM_STATUS)
    {
        case delphi_srr_msgs::SrrStatus5::CAN_TX_SYSTEM_STATUS_Running:
            status_.status = cav_msgs::DriverStatus::OPERATIONAL;
            break;
        case delphi_srr_msgs::SrrStatus5::CAN_TX_SYSTEM_STATUS_Faulty:
            status_.status = cav_msgs::DriverStatus::FAULT;
            break;
        case delphi_srr_msgs::SrrStatus5::CAN_TX_SYSTEM_STATUS_Blocked:
        case delphi_srr_msgs::SrrStatus5::CAN_TX_SYSTEM_STATUS_Hot:
            status_.status = cav_msgs::DriverStatus::DEGRADED;
            break;
        default:
            status_.status = cav_msgs::DriverStatus::OFF;
            break;
    }
}
