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

#include "dsrc_client.h"

#include <driver_application/driver_application.h>

#include <boost/asio.hpp>
#include <boost/signals2/signal.hpp>
#include <boost/lambda/lambda.hpp>

#include <memory>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <queue>
#include <vector>

#include <cav_msgs/ByteArray.h>
#include <cav_srvs/SendMessage.h>

#include <dynamic_reconfigure/server.h>
#include <dsrc/DSRCConfig.h>

#include <ros/ros.h>

#include <map>
#include <set>

/**
 * @class DSRCApplication
 * @brief Is the class responsible for the ROS dsrc driver
 */
class DSRCApplication : public cav::DriverApplication
{
private:
    struct WaveConfigStruct
    {
        std::string name, psid, dsrc_id, channel, priority;

        WaveConfigStruct(){};
        WaveConfigStruct(const std::string &name,
                         const std::string &psid,
                         const std::string &dsrc_id,
                         const std::string &channel,
                         const std::string &priority) : name(name),
                                         psid(psid),
                                         dsrc_id(dsrc_id),
                                         channel(channel),
                                         priority(priority) {}
    };

public:


    /**
     * @brief constructor
     * @param argc - command line argument count
     * @param argv - command line arguments
     */
    DSRCApplication(int argc, char** argv);

    ~DSRCApplication() { shutdown(); }

private:

    std::vector<std::string> api_;

    //ROS
    ros::Publisher comms_pub_;
    ros::Subscriber comms_sub_;
    ros::ServiceServer comms_srv_;
    std::shared_ptr<ros::NodeHandle> comms_api_nh_;

    //dynamic reconfig
    dsrc::DSRCConfig config_;
    std::mutex cfg_mutex_;
    std::shared_ptr<dynamic_reconfigure::Server<dsrc::DSRCConfig>> dyn_cfg_server_;
    boost::recursive_mutex dyn_cfg_mutex_;

    std::deque<std::shared_ptr<std::vector<uint8_t>>> send_msg_queue_;
    bool connecting_ = false;
    std::shared_ptr <std::thread> connect_thread_;

    std::vector<WaveConfigStruct> wave_cfg_items_;

    DSRCOBUClient dsrc_client_;
    boost::system::error_code dsrc_client_error_;
    uint32_t output_queue_size_;

    /**
     * @brief Initializes ROS context for this node
     *
     * Establishes the connection to the DSRC hardware. Sets up pertinent events and corresponding topics
     */
    virtual void initialize() override;

    /**
     * @brief Called by the base DriverApplication class after spin
     *
     * Sends messages from the outgoing queue
     */
    virtual void post_spin() override;

    /**
     * @brief Called by the base DriverApplication class prior to Spin
     *
     * Manages local state of hardware device, reconnecting as needed
     */
    virtual void pre_spin() override;

    virtual void shutdown() override;

    /**
     * @brief Called by the base DriverApplication class to fetch this implementation's api
     *
     * The API is a list of fully scoped names to topics and services specified by the
     * CAV Platform architecture
     *
     * @return list of api
     */
    inline virtual std::vector<std::string>& get_api() override  { return api_; }

    /**
    * @brief Handles the DSRC onConnect Event
    *
    * Establishes status of the node
    */
    void onConnectHandler();

    /**
    * @brief Handles the DSRC onDisconnect Event
    *
    * On Disconnect this node will enter a reconnect loop attempting to reconnect
    */
    void onDisconnectHandler();

    /**
    * @brief Handles messages received from the DSRCDSRCOBUClient
    *
    * Populates a ROS message with the contents of the incoming OBU message, and
    * publishes to the ROS 'inbound_binary_msg' topic.
    */
    void onMessageReceivedHandler(const std::vector<uint8_t> &data, uint16_t id);

    /**
    * @brief Packs an outgoing message into J2375 standard.
    * @param message
    *
    * This processes an incoming ByteArray message, and packs it according to the
    * Active Message file for the OSU.
    */
    std::vector<uint8_t> packMessage(const cav_msgs::ByteArray& message);

    /**
     * @brief Handles outbound messages from the ROS network
     * @param message
     *
     * This method packs the message according to the J2375 2016 standard,
     * and sends it to the client program
     */
    void onOutboundMessage(const cav_msgs::ByteArray::ConstPtr& message);

    /**
     * @brief Message sending service
     * @param req
     * @param res
     *
     */
    bool sendMessageSrv(cav_srvs::SendMessage::Request& req, cav_srvs::SendMessage::Response& res);

    /**
     * @brief Sends a message from the queue of outbound messages
     */
    void sendMessageFromQueue();

    /**
     * @brief Callback for dynamic reconfig service
     * @param cfg
     * @param level
     */
    void dynReconfigCB(dsrc::DSRCConfig & cfg, uint32_t level);


    /**
     * @brief Loads the wave file with the given name for configuring WAVE message ids with channel and PSID
     * @param fileName
     */
    void loadWaveConfig(const std::string& fileName);

    /**
     * @brief converts a uint8_t vector to an ascii representation
     * @param v
     * @return
     */
    std::string uint8_vector_to_hex_string(const std::vector<uint8_t>& v);
    
};