#pragma once

/*
 * Copyright (C) 2019-2022 LEIDOS.
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

#include "ns-3_client.h"
#include <boost/asio.hpp>
#include <boost/signals2/signal.hpp>
#include <boost/lambda/lambda.hpp>
#include <driver_application/driver_application.h>
#include <memory>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <queue>
#include <vector>

#include <cav_msgs/ByteArray.h>
#include <cav_srvs/SendMessage.h>
#include <geometry_msgs/PoseStamped.h>


#include <dynamic_reconfigure/server.h>

#include <ros/ros.h>

#include <map>
#include <set>

/**
 * @class NS3Adapter
 * @brief Is the class responsible for the NS-3 Adapter driver
 */
class NS3Adapter : public cav::DriverApplication
{
    private:
    struct WaveConfigStruct
    {
        std::string name, psid, ns3_id, channel, priority;

        WaveConfigStruct(){};
        WaveConfigStruct(const std::string &name,
                         const std::string &psid,
                         const std::string &ns3_id,
                         const std::string &channel,
                         const std::string &priority) : name(name),
                                         psid(psid),
                                         ns3_id(ns3_id),
                                         channel(channel),
                                         priority(priority) {}
    };

    public:

        /**
        * @brief constructor
        * @param argc - command line argument count
        * @param argv - command line arguments
        */
        NS3Adapter(int argc, char** argv);

        ~NS3Adapter() { shutdown(); }

    private:

        std::vector<std::string> api_;

        //ROS
        ros::Publisher comms_pub_;
        ros::Subscriber comms_sub_;
        ros::Subscriber pose_sub_;
        ros::ServiceServer comms_srv_;
        std::shared_ptr<ros::NodeHandle> comms_api_nh_;

        // Current vehicle pose in map
        geometry_msgs::PoseStamped pose_msg_;

        std::string vehicle_id_;


        //dynamic reconfig
        std::mutex cfg_mutex_;
        //std::shared_ptr<dynamic_reconfigure::Server<dsrc::DSRCConfig>> dyn_cfg_server_;
        boost::recursive_mutex dyn_cfg_mutex_;
        NS3Client ns3_client_;

        std::deque<std::shared_ptr<std::vector<uint8_t>>> send_msg_queue_;
        bool connecting_ = false;
        std::shared_ptr <std::thread> connect_thread_;
        boost::system::error_code ns3_client_error_;

        std::vector<WaveConfigStruct> wave_cfg_items_;
        uint32_t queue_size_;

        /**
        * @brief Initializes ROS context for this node
        *
        * Establishes the connection to the NS-3 Adapter. Sets up pertinent events and corresponding topics
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


    public: 
        /**
        * @brief Handles the NS-3 onConnect Event
        *
        * Establishes status of the node
        */
        void onConnectHandler();

        /**
        * @brief Handles the NS-3 onDisconnect Event
        *
        * On Disconnect this node will enter a reconnect loop attempting to reconnect
        */
        void onDisconnectHandler();

        /**
        * @brief Handles messages received from the NS-3 Client
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
        void onOutboundMessage(const cav_msgs::ByteArrayPtr& message);

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
        *
        void dynReconfigCB(dsrc::DSRCConfig & cfg, uint32_t level);*/


        /**
        * @brief Loads the wave file with the given name for configuring WAVE message ids with channel and PSID
        * @param fileName
        */
        void loadWaveConfig(const std::string& fileName);

        void pose_cb(geometry_msgs::PoseStamped pose_msg);

        cav_msgs::DriverStatus getDriverStatus();

        std::deque<std::shared_ptr<std::vector<uint8_t>>> getMsgQueue();

        /**
        * @brief converts a uint8_t vector to an ascii representation
        * @param v
        * @return
        */
        std::string uint8_vector_to_hex_string(const std::vector<uint8_t>& v);
};