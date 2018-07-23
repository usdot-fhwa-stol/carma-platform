#pragma once
/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Torc Robotics, LLC
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Torc Robotics, LLC nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */


#include <cav_srvs/Bind.h>
#include <cav_srvs/GetDriverApi.h>
#include <cav_srvs/GetDriverStatus.h>
#include <cav_msgs/DriverStatus.h>
#include <cav_msgs/SystemAlert.h>

#include <bondcpp/bond.h>

#include <unordered_map>
#include <mutex>

namespace cav
{

/**
 * @class DriverApplication driver_application.h
 * @brief Base class for CAV Platform drivers
 *
 * DriverApplication is responsible for common behaviours of the drivers in the CAV Platform Architecture. It initiliazes
 * the ROS node context and is responsible for managing the ROS "spin" loop. *
 */
class DriverApplication
{
public:

    /**
     *
     * @param argc - Command line argument count
     * @param argv - Array of command line arguments
     * @param name - Node name
     */
    DriverApplication(int argc, char** argv, std::string name = "driver") : status_(), spin_rate(50),shutdown_(false)
    {
        ros::init(argc,argv,name);
    }


    virtual ~DriverApplication(){};

    /**
     * @brief Starts the application
     *
     * @return 0 on exit with no errors
     */
    virtual int run() final;

protected:

    /**
     * @brief Sets the status of the DriverApplication
     *
     * If status is changed then the DriverApplication will release all bonds to notify to connected nodes
     * that the status has been updated. The name is managed by the DriverApplication class and is not overriden
     * within a call to this function
     *
     *  @param status - The status to be set
     */
    virtual void setStatus(cav_msgs::DriverStatus status) final;

    /**
     * @brief Get's status
     * @return
     */
    virtual cav_msgs::DriverStatus getStatus() final { return status_; }

    /**
     * @var spin_rate
     * @brief should be set prior to DriverApplication beginning its control loop. Ideally inheriting classes
     * should set the spin_rate be set within the DriverApplication::initialize() function
     */
    int spin_rate;

    /**
     * @var pnh_
     * @brief ROS NodeHandle within the private  ("~") namespace
     */

    /**
     * @var nh_
     * @brief ROS NodeHandle within the node root namespace
     */
    std::shared_ptr<ros::NodeHandle> pnh_, nh_;

private:

    /**
     * @brief Called every second to publish the status message
     */
    void status_publish_timer(const ros::TimerEvent &) const;

    /**
     * @brief Callback for bind service
     *
     * This node will bind on /[driver_name]/bond topic with id passed in the request
     * @param req
     * @param res
     * @return
     */
    bool bind_service_cb(cav_srvs::Bind::Request &req, cav_srvs::Bind::Response &res);

    /**
     * @brief Callback go the get_driver_api service
     *
     * This function will return a list of API specified by the inheriting class through
     * get_api
     * @param req
     * @param res
     * @return true to signal we handled this callback
     */
    bool get_driver_api_cb(cav_srvs::GetDriverApiRequest &req,
                                  cav_srvs::GetDriverApiResponse &res);


    /**
     * @brief Callback for the get_status service
     *
     * This function will return the current status of the driver_node
     * @param req - empty
     * @param res - contains driver status of this node
     * @return true to signal we handled this calback
     */
    bool get_status_cb(cav_srvs::GetDriverStatusRequest& req,cav_srvs::GetDriverStatusResponse& res);

    /**
     * @brief Returns the fully scoped API for this node
     *
     * @return
     */
    virtual std::vector<std::string>& get_api() = 0;

    /**
     * @brief Initialize the node prior to beginning ROS loop.
     *
     * This function is called after the DriverApplication has initialized the ROS context and nodehandles
     *
     * @return
     */
    virtual void initialize() = 0;

    /**
     * @brief Called prior to spinOnce()
     *
     */
    virtual void pre_spin() = 0;

    /**
     *@brief Called after spinOnce()
     */
    virtual void post_spin() = 0;

    virtual void shutdown(){((void)0);};

    void system_alert_cb(const cav_msgs::SystemAlertConstPtr& msg);


    ros::Publisher driver_status_pub_;
    ros::Subscriber system_alert_sub_;
    bool shutdown_;
    cav_msgs::DriverStatus status_;
    std::unordered_map<std::string, std::shared_ptr<bond::Bond>> bond_map_;
    std::mutex bond_mutex_;
};

};

