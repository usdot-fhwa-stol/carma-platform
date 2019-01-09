#pragma once

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

#include <driver_wrapper/driver_wrapper.h>
#include <delphi_srr_msgs/SrrStatus5.h>

class DelphiSrr2RadarDriverWrapper : public cav::DriverWrapper
{
    ros::Subscriber srr_status5_sub_;

public:
    DelphiSrr2RadarDriverWrapper(int argc, char **argv, const std::string &name = "delphi_srr2_radar_driver_wrapper");
    virtual ~DelphiSrr2RadarDriverWrapper();

private:

    /**
     * @brief Callback for handling srr_status5 message
     */
    void srr_status5_cb(const delphi_srr_msgs::SrrStatus5ConstPtr& msg);

    //cav::DriverWrapper members
    virtual void initialize();
    virtual void pre_spin();
    virtual void post_spin();
    virtual void shutdown();

};
