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
#include "object_tracker.h"
#include "twist_history_buffer.h"
#include <sensor_fusion/SensorFusionConfig.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/TwistStamped.h>
#include <cav_msgs/HeadingStamped.h>
#include <cav_msgs/ExternalObjectList.h>
#include <cav_msgs/BSM.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <boost/bind.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/random_generator.hpp>
#include <boost/uuid/uuid_io.hpp>

#include <bondcpp/bond.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <dynamic_reconfigure/server.h>

#include <ros/ros.h>

#include <string>
#include <unordered_map>
#include <memory>




/**
 * @brief A ROS node that monitors multiple sources to produce a filtered version
 *
 * This class monitors all drivers that provide position and tracked objects api*
 *
 */
class SensorFusionApplication
{
public:

    /**
     * @brief Initializes the ROS context of this node
     * @param argc Command line argument count
     * @param argv Command line arguments
     * @param name Name of the node
     */
    SensorFusionApplication(int argc, char** argv, std::string name="sensor_fusion") : twist_history_buffer_(ros::Duration(2.0))
    {
        ros::init(argc, argv, name);
        dyn_cfg_server_.reset(new dynamic_reconfigure::Server<sensor_fusion::SensorFusionConfig>());
    }


    /**
     * @brief Main process function
     * @return 0 on Success
     */
    int run();

private:

    std::unique_ptr<dynamic_reconfigure::Server<sensor_fusion::SensorFusionConfig>> dyn_cfg_server_;
    sensor_fusion::SensorFusionConfig config_;
    std::unique_ptr<torc::ObjectTracker> tracker_;
    tf2_ros::Buffer tf2_buffer_;
    ros::Timer update_services_timer_;
    std::unique_ptr<tf2_ros::TransformListener> tf2_listener_;
    cav::TwistHistoryBuffer twist_history_buffer_;

    std::unique_ptr<ros::NodeHandle> nh_, pnh_;
    std::unordered_map<std::string, std::unique_ptr<bond::Bond>> bond_map_;
    bool use_interface_mgr_;
    std::string inertial_frame_name_, body_frame_name_, ned_frame_name_;
    /**
     * @brief This function is the bond call back for on_broken event    *
     * @param node_name name of the node this callback is firing for
     *
     * When a bond is broken , we clear our record of the bond destroying our reference to the bond
     */
    void on_broken_cb(const std::string& node_name)
    {
        bond_map_.erase(node_name);
    }

    /**
     * @brief This functions is the bond callback for on_connected event
     * @param node_name name of the node this callback is firing for
     *
     * Right now this is only used for debugging
     */

    void on_connected_cb(const std::string &node_name)
    {
        ROS_DEBUG_STREAM("Bonded to " << node_name);
    }


    /**
     * @brief Callback for dynamic_reconfigure. This is called by the dynamic_reconfigure server.
     *
     * @param cfg
     * @param level
     */
    void dyn_recfg_cb(sensor_fusion::SensorFusionConfig &cfg, uint32_t level);


    /**
     * @brief Handles the call to the interface manager to receive api of the given node
     *
     * This function queries the interface manager for a give api specified by name.
     * After receiving a reply from the interface manager it builds a list of FQN of topics
     * that have not been subscribed to.
     *
     * @param type The driver_type we are querying
     * @param name The name of the service to query
     * @return A vector containing the FQN of the services queried
     */
    std::vector<std::string> get_api(const std::string& name);

    std::unordered_map<std::string, ros::Subscriber> sub_map_;
    /**
     * @brief Manages subscribing to driver services
     *
     * This should be called periodically to maintain updates
     */
    void update_subscribed_services();


    ros::Publisher odom_pub_, navsatfix_pub_, heading_pub_, velocity_pub_, objects_pub_, vehicles_pub_;

    /**
     * @brief Publishes the filtered updates
     *
     * At the moment since this is just a skeleton application
     * we are only re-publishing the receive data
     *
     * Later this will publish processed data
     */
    void publish_updates();


    /**
     * The following callbacks are attached to ROS topics with their provided type
     *
     * todo: make the callbacks to real work
     * These callbacks currently for the skeleton only store the message in a map
     * to be used later
     */

    std::unordered_map<std::string, nav_msgs::OdometryConstPtr> odom_map_;
    void odom_cb(const ros::MessageEvent<nav_msgs::Odometry>& event);

    std::unordered_map<std::string, sensor_msgs::NavSatFixConstPtr> navsatfix_map_;
    void navsatfix_cb(const ros::MessageEvent<sensor_msgs::NavSatFix>& event);

    std::unordered_map<std::string, cav_msgs::HeadingStampedConstPtr> heading_map_;
    void heading_cb(const ros::MessageEvent<cav_msgs::HeadingStamped>& event);

    std::unordered_map<std::string, geometry_msgs::TwistStampedConstPtr> velocity_map_;
    void velocity_cb(const ros::MessageEvent<geometry_msgs::TwistStamped>& event);

    std::deque<std::pair<std::string, cav_msgs::ExternalObjectListConstPtr>> objects_cb_q_;
    void objects_cb(const cav_msgs::ExternalObjectListConstPtr &objList,const std::string&);

    void bsm_cb(const cav_msgs::BSMConstPtr& msg);
};

