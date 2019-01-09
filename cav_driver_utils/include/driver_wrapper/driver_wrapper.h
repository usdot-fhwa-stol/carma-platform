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

#include <ros/ros.h>
#include <cav_msgs/DriverStatus.h>
#include <cav_msgs/SystemAlert.h>

namespace cav
{

class DriverWrapper
{

public:

    /**
     * @brief DriverWrapper Constructor
     * @param argc Command line argument count
     * @param argv Array of command line arguments
     * @param name Node name
     */
    DriverWrapper(int argc, char** argv, std::string name = "driver_wrapper") : status_(), spin_rate_(10), shutdown_(false)
    {
        ros::init(argc, argv, name);
    }

    /**
     * @brief ~DriverWrapper Destructor
     */
    virtual ~DriverWrapper(){}

    /**
     * @brief run Starts the driver wrapper
     * @return 0 on exit with no errors
     */
    virtual int run() final;

protected:

    std::shared_ptr<ros::NodeHandle> nh_, private_nh_;
    cav_msgs::DriverStatus status_;

private:

    /**
     * @brief Initialize the node prior to beginning ROS loop.
     */
    virtual void initialize() = 0;

    /**
     * @brief Called prior to spinOnce()
     */
    virtual void pre_spin() = 0;

    /**
     * @brief Called after spinOnce()
     */
    virtual void post_spin() = 0;

    /**
     * @brief Prepare for node shutdown
     */
    virtual void shutdown() = 0;

    /**
     * @brief Called every second to publish the status message
     */
    void status_publish_timer(const ros::TimerEvent &) const;

    /**
     * @brief Callback on system alert topic
     */
    void system_alert_cb(const cav_msgs::SystemAlertConstPtr& msg);

    // Initialize necessary publishers and subscribers
    ros::Publisher  driver_status_pub_;
    ros::Subscriber system_alert_sub_;
    int spin_rate_;
    bool shutdown_;

};

};
