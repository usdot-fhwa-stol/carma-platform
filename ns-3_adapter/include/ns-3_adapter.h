#pragma once

#include "ns-3_client.h"
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

#include <ros/ros.h>

#include <map>
#include <set>

/**
 * @class NS3Adapter
 * @brief Is the class responsible for the NS-3 Adapter driver
 */
class NS3Adapter : public cav::DriverApplication
{

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
    ros::ServiceServer comms_srv_;

    //dynamic reconfig
    std::mutex cfg_mutex_;
    std::shared_ptr<dynamic_reconfigure::Server<dsrc::DSRCConfig>> dyn_cfg_server_;
    boost::recursive_mutex dyn_cfg_mutex_;

    std::deque<std::shared_ptr<std::vector<uint8_t>>> send_msg_queue_;
    bool connecting_ = false;
    std::shared_ptr <std::thread> connect_thread_;

    std::vector<WaveConfigStruct> wave_cfg_items_;
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