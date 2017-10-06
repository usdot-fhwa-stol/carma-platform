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

#include "dsrc_application.h"
#include <rapidjson/document.h>
#include <rapidjson/istreamwrapper.h>
#include <rapidjson/schema.h>
#include <fstream>

std::string DSRCApplication::uint8_vector_to_hex_string(const std::vector<uint8_t>& v) {
    std::stringstream ss;
    ss << std::hex << std::setfill('0');
    std::vector<uint8_t>::const_iterator it;
    for (it = v.begin(); it != v.end(); it++) {
        ss << std::setw(2) << static_cast<unsigned>(*it);
    }

    return ss.str();
}

DSRCApplication::DSRCApplication(int argc, char **argv) : cav::DriverApplication(argc, argv, "dsrc")
{
    output_queue_size_ = 1000;
    cav_msgs::DriverStatus status;
    status.status = cav_msgs::DriverStatus::OFF;
    status.comms = true;
    setStatus(status);
}


void DSRCApplication::initialize() {
    std::string wave_cfg_file;
    ros::NodeHandle pnh("~");
    pnh.param<std::string>("wave_cfg_file",wave_cfg_file,"etc/wave.json");
    pnh.param<int>("listening_port",config_.listening_port, 5398);
    pnh.param<int>("dsrc_listening_port",config_.dsrc_listening_port, 1516);
    pnh.param<std::string>("dsrc_address",config_.dsrc_address, "169.254.1.1");
    loadWaveConfig(wave_cfg_file);
    comms_api_nh_.reset(new ros::NodeHandle("~comms"));
    dyn_cfg_server_.reset(new dynamic_reconfigure::Server<dsrc::DSRCConfig>(dyn_cfg_mutex_));
    dyn_cfg_server_->updateConfig(config_);
    dyn_cfg_server_->setCallback([this](dsrc::DSRCConfig & cfg, uint32_t level) { dynReconfigCB(cfg, level); });

    //Setup connection handlers
    dsrc_client_error_.clear();
    dsrc_client_.onConnect.connect([this]() { onConnectHandler(); });
    dsrc_client_.onDisconnect.connect([this]() { onDisconnectHandler(); });
    dsrc_client_.onError.connect([this](const boost::system::error_code& err){dsrc_client_error_ = err;});


    //Setup the ROS API
    std::string node_name = ros::this_node::getName();
    api_.clear();
    
    //Comms Subscriber
    comms_sub_ = comms_api_nh_->subscribe("outbound", output_queue_size_, &DSRCApplication::onOutboundMessage, this);
    api_.push_back(comms_sub_.getTopic());

    //Comms Publisher
    comms_pub_ = comms_api_nh_->advertise<cav_msgs::ByteArray>("recv", 1000);
    api_.push_back(comms_pub_.getTopic());

    //Comms Service
    comms_srv_ = comms_api_nh_->advertiseService("send", &DSRCApplication::sendMessageSrv, this);
    api_.push_back(comms_srv_.getService());

    dsrc_client_.onMessageReceived.connect([this](std::vector<uint8_t> const &msg, uint16_t id) {onMessageReceivedHandler(msg, id); });
    
    spin_rate = 50;
}

void DSRCApplication::onConnectHandler() {
    ROS_INFO_STREAM("DSRC Driver Connected");
    cav_msgs::DriverStatus status = getStatus();
    status.status = cav_msgs::DriverStatus::OPERATIONAL;
    setStatus(status);
}

void DSRCApplication::onDisconnectHandler() {
    cav_msgs::DriverStatus status = getStatus();
    status.status = cav_msgs::DriverStatus::OFF;
    setStatus(status);
    ROS_WARN_STREAM("DSRC Driver Disconnected");
}

/**
* @brief Handles messages received from the DSRCOBUClient
*
* Populates a ROS message with the contents of the incoming OBU message, and
* publishes to the ROS 'recv' topic.
*/
void DSRCApplication::onMessageReceivedHandler(const std::vector<uint8_t> &data, uint16_t id) {
    // Create and populate the message
    auto it = std::find_if(wave_cfg_items_.begin(),wave_cfg_items_.end(),[id](const WaveConfigStruct& entry)
                                                                            {
                                                                               return entry.dsrc_id == std::to_string(id);
                                                                            });

    cav_msgs::ByteArray msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "";
    msg.messageType = it != wave_cfg_items_.end() ? it->name : "Unknown";
    msg.content = data;
    // Publish it
    comms_pub_.publish(msg);

    ROS_DEBUG("Application received Data: %ld bytes, message: %s", data.size(), uint8_vector_to_hex_string(data).c_str());
}

/**
 * @brief Packs an outgoing message into J2375 standard.
 * @param message
 *
 * This processes an incoming ByteArray message, and packs it according to the
 * J2735 standard.
 *
 * TODO: Right now it doesn't do anything except return the message data (ignoring message type).
 * Depending on what the input is, that might be all that's necessary, but possibly more.
 * Depending on what the input is, that might be all that's necessary, but possibly more.
 */
std::vector<uint8_t> DSRCApplication::packMessage(const cav_msgs::ByteArray& message) {
    std::stringstream ss;
    auto wave_item = std::find_if(wave_cfg_items_.begin(),wave_cfg_items_.end(),[&message](const WaveConfigStruct& entry)
    {
        return entry.name == message.messageType;
    });

    WaveConfigStruct cfg;
    if(wave_item == wave_cfg_items_.end())
    {
        ROS_WARN_STREAM("No wave config entry for type: " << message.messageType << ", using defaults");
        cfg.name = message.messageType;
        cfg.channel = 178;
        cfg.priority = 1;
        cfg.dsrc_id = (message.content[0] << 8 ) | message.content[1];
        cfg.psid = cfg.dsrc_id;
    }
    else
    {
        cfg = *wave_item;
    }

    ss << "Version=0.7" << std::endl;
    ss << "Type=" << cfg.name << std::endl;
    ss << "PSID=" << cfg.psid << std::endl;
    ss << "Priority=" << cfg.priority << std::endl;
    ss << "TxMode=ALT" << std::endl;
    ss << "TxChannel=" << cfg.channel << std::endl;
    ss << "TxInterval=0" << std::endl;
    ss << "DeliveryStart=" << std::endl;
    ss << "DeliveryStop=" << std::endl;
    ss << "Signature=False" << std::endl;
    ss << "Encryption=False" << std::endl;

    ss << "Payload=" <<  uint8_vector_to_hex_string(message.content) << std::endl;

    std::string str = ss.str();

    return std::vector<uint8_t>(str.begin(), str.end());
}

/**
* @brief Handles outbound messages from the ROS network
* @param message
*
* This method receives a message from the ROS network, and adds it to the send queue.
*/
void DSRCApplication::onOutboundMessage(const cav_msgs::ByteArray::ConstPtr& message) {
    if(!dsrc_client_.connected())
    {
        ROS_WARN_STREAM("Outbound message received but node is not connected to DSRC Radio");
        return;
    }
    std::shared_ptr<std::vector<uint8_t>> message_content = std::make_shared<std::vector<uint8_t>>(std::move(packMessage(*message)));
    send_msg_queue_.push_back(std::move(message_content));
}

/**
* @brief Sends a message from the queue of outbound messages
*/
void DSRCApplication::sendMessageFromQueue() {
    if (!send_msg_queue_.empty()) {
        ROS_DEBUG_STREAM("Sending message: " << std::string(send_msg_queue_.front()->begin(),send_msg_queue_.front()->end()));
        bool success = dsrc_client_.sendDsrcMessage(send_msg_queue_.front());
        send_msg_queue_.pop_front();
        if (!success) {
            ROS_WARN_STREAM("Message send failed");
        }
        else {
            ROS_DEBUG("Message successfully sent from queue");
        }
    }
}

/**
* @brief Message sending service
* @param req
* @param res
*/
bool DSRCApplication::sendMessageSrv(cav_srvs::SendMessage::Request& req, cav_srvs::SendMessage::Response& res) {
    if(!dsrc_client_.connected())
    {
        ROS_WARN_STREAM("Outbound message received but node is not connected to DSRC Radio");
        res.errorStatus = 1;
        return true;
    }

    // Package data into a message shared pointer; this lets packMessage have
    // the same interface for outgoing messages from the topic and service.
    const cav_msgs::ByteArray::ConstPtr message = cav_msgs::ByteArray::ConstPtr(new cav_msgs::ByteArray(req.message_to_send));
    std::shared_ptr<std::vector<uint8_t>> message_data = std::make_shared<std::vector<uint8_t>>(std::move(packMessage(*message)));
    bool success = dsrc_client_.sendDsrcMessage(message_data);
    if (success) {
        ROS_DEBUG("SendMessage service returned success");
        res.errorStatus = 0;
    }
    else {
        ROS_WARN_STREAM("SendMessage service returned failure");
        res.errorStatus = 1;
    }

    return true;
}

void DSRCApplication::pre_spin()
{
    // Adjust output queue size if config changed.
    if(dsrc_client_error_)
    {
        ROS_ERROR_STREAM("DSRC Client Error : " << dsrc_client_error_.message());
        dsrc_client_.close();
        dsrc_client_error_.clear();
    }
    //If we are not connected
    if (!connecting_ && !dsrc_client_.connected())
    {
        connecting_ = true;
        if (connect_thread_)
            connect_thread_->join();

        //We don't want to block the spin thread because the driver
        //application maintains driver status topic
        connect_thread_.reset(new std::thread([this]()
        {
            dsrc::DSRCConfig cfg;
            {
                std::lock_guard<std::mutex> lock(cfg_mutex_);
                cfg = config_;
            }
            ROS_INFO("Attempting to connect to OBU");
            boost::system::error_code ec;
            ROS_INFO("Connecting to %s:%u", cfg.dsrc_address.c_str(), cfg.dsrc_listening_port);
            ROS_INFO("Local port: %u", cfg.listening_port);
            try {
                if (!dsrc_client_.connect(cfg.dsrc_address, cfg.dsrc_listening_port,
                                          cfg.listening_port, ec))
                {
                    ROS_WARN_STREAM("Failed to connect, err: " << ec.message());
                }
            }catch(std::exception e)
            {
                ROS_ERROR_STREAM("Exception connecting to dsrc radio: " << e.what() << " error_code: " << ec.message());
                ROS_ERROR_STREAM("Config:\n\tdsrc_address:" << cfg.dsrc_address
                                         << "\n\tdsrc_listening_port:" << cfg.dsrc_listening_port
                                         << "\n\tlistening_port:" << cfg.listening_port);
            }


            connecting_ = false;
        }));
    }
}


void DSRCApplication::post_spin() {
    sendMessageFromQueue();
}

void DSRCApplication::dynReconfigCB(dsrc::DSRCConfig & cfg, uint32_t level)
{
    std::lock_guard<std::mutex> lock(cfg_mutex_);
    if(config_.dsrc_address != cfg.dsrc_address ||
       config_.dsrc_listening_port != cfg.dsrc_listening_port ||
       config_.listening_port != cfg.listening_port)
    {
        ROS_INFO_STREAM("DynReconfig dsrc_address " << cfg.dsrc_address
                                                     << " dsrc_listening_port "
                                                     << cfg.dsrc_listening_port
                                                     << " listening_port "
                                                     << cfg.listening_port);
        config_ = cfg;
        dsrc_client_.close();
    }
}

void DSRCApplication::loadWaveConfig(const std::string &fileName)
{
    ROS_DEBUG_STREAM("Loading wave config");
    const char* schema = "{\n"
                         " \"$schema\":\"http://json-schema.org/draft-06/schema\",\n"
                         " \"title\":\"Wave Config Schema\",\n"
                         " \"description\":\"A simple schema to describe DSRC/Wave messages\",\n"
                         "  \"type\": \"array\",\n"
                         "  \"items\": {\n"
                         "    \"type\": \"object\",\n"
                         "    \"properties\": {\n"
                         "      \"name\": {\n"
                         "        \"description\": \"message type - abbreviated name\",\n"
                         "        \"type\": \"string\"\n"
                         "      },\n"
                         "      \"psid\": {\n"
                         "        \"description\": \"psid assigned to message type in decimal\",\n"
                         "        \"type\": \"string\"\n"
                         "      },\n"
                         "      \"dsrc_id\": {\n"
                         "        \"description\": \"J2735 DSRC id assigned to message type in decimal\",\n"
                         "        \"type\": \"string\"\n"
                         "      },\n"
                         "      \"channel\": {\n"
                         "        \"description\": \"DSRC radio channel assigned to message type in decimal\",\n"
                         "        \"type\": \"string\"\n"
                         "      },\n"
                         "      \"priority\": {\n"
                         "        \"description\": \"WSM Priotiy to use assigned to message type in decimal\",\n"
                         "        \"type\":\"string\"\n"
                         "      }\n"
                         "    },\n"
                         "    \"required\":[\"name\",\"psid\",\"dsrc_id\",\"channel\",\"priority\"]"
                         "  }\n"
                         "}\n";

    std::ifstream file;
    try
    {
        file.open(fileName);
    }
    catch (std::exception& e)
    {
        ROS_ERROR_STREAM("Unable to open file : " << fileName << ", exception: " << e.what());
        return;
    }

    rapidjson::Document sd;
    if(sd.Parse(schema).HasParseError())
    {
        ROS_ERROR_STREAM("Invalid Wave Config Schema");
        return;
    }

    rapidjson::SchemaDocument schemaDocument(sd);
    rapidjson::Document doc;
    rapidjson::IStreamWrapper isw(file);
    if(doc.ParseStream(isw).HasParseError())
    {
        ROS_ERROR_STREAM("Error Parsing Wave Config");
        return;
    }

    rapidjson::SchemaValidator validator(schemaDocument);
    if(!doc.Accept(validator))
    {
        ROS_ERROR_STREAM("Wave Config improperly formatted");
        return;
    }

    for(auto& it : doc.GetArray())
    {
        auto entry = it.GetObject();
        wave_cfg_items_.emplace_back(entry["name"].GetString(),
                                     entry["psid"].GetString(),
                                     entry["dsrc_id"].GetString(),
                                     entry["channel"].GetString(),
                                     entry["priority"].GetString());

    }
}

void DSRCApplication::shutdown()
{
    ROS_INFO("Shutdown signal received from DriverApplication");
    if (connect_thread_)
    {
        ROS_INFO("Cleaning up connection thread");
        connect_thread_->join();
        connect_thread_.reset();
    }

    ROS_INFO("Closing connection to radio");
    dsrc_client_.close();

}
