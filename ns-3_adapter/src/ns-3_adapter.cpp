#include "ns-3_adapter.h"

#include <rapidjson/document.h>
#include <rapidjson/istreamwrapper.h>
#include <rapidjson/schema.h>
#include <cav_msgs/ByteArray.h>
#include <fstream>

std::string NS3Adapter::uint8_vector_to_hex_string(const std::vector<uint8_t>& v) {
    std::stringstream ss;
    ss << std::hex << std::setfill('0');
    std::vector<uint8_t>::const_iterator it;
    for (it = v.begin(); it != v.end(); it++) {
        ss << std::setw(2) << static_cast<unsigned>(*it);
    }

    return ss.str();
}

NS3Adapter::NS3Adapter(int argc, char **argv) : cav::DriverApplication(argc, argv, "ns3")
{
    queue_size_ = 1000;
    cav_msgs::DriverStatus status;
    status.status = cav_msgs::DriverStatus::OFF;
    status.comms = true;
    setStatus(status);
}


void NS3Adapter::initialize() {
    std::string wave_cfg_file;
    ros::NodeHandle pnh("~");
    pnh.param<std::string>("wave_cfg_file",wave_cfg_file,"etc/wave.json");
    //pnh.param<int>("listening_port",config_.listening_port, 5398);
    //pnh.param<int>("dsrc_listening_port",config_.dsrc_listening_port, 1516);
    //pnh.param<std::string>("dsrc_address",config_.dsrc_address, "169.254.1.1");    
    loadWaveConfig(wave_cfg_file);
    //comms_api_nh_.reset(new ros::NodeHandle("comms"));
    //dyn_cfg_server_.reset(new dynamic_reconfigure::Server<dsrc::DSRCConfig>(dyn_cfg_mutex_));
    //dyn_cfg_server_->updateConfig(config_);
    //dyn_cfg_server_->setCallback([this](dsrc::DSRCConfig & cfg, uint32_t level) { dynReconfigCB(cfg, level); });

    //Setup connection handlers
    ns3_client_error_.clear();
    ns3_client_.onConnect.connect([this]() { onConnectHandler(); });
    ns3_client_.onDisconnect.connect([this]() { onDisconnectHandler(); });
    ns3_client_.onError.connect([this](const boost::system::error_code& err){ns3_client_error_ = err;});


    //Setup the ROS API
    std::string node_name = ros::this_node::getName();
    api_.clear();
    
    //Comms Subscriber
    comms_sub_ = comms_api_nh_->subscribe("outbound_binary_msg", queue_size_, &NS3Adapter::onOutboundMessage, this);
    api_.push_back(comms_sub_.getTopic());

    //Comms Publisher
    comms_pub_ = comms_api_nh_->advertise<cav_msgs::ByteArray>("inbound_binary_msg", queue_size_);
    api_.push_back(comms_pub_.getTopic());

    //Comms Service
    comms_srv_ = comms_api_nh_->advertiseService("send", &NS3Adapter::sendMessageSrv, this);
    api_.push_back(comms_srv_.getService());

    pose_sub_ = pnh_->subscribe("current_pose", 1, &NS3Adapter::pose_cb, this);

    pnh_->getParam("vehicle_id", vehicle_id_);


    ns3_client_.onMessageReceived.connect([this](std::vector<uint8_t> const &msg, uint16_t id) {onMessageReceivedHandler(msg, id); });
    
    spin_rate = 50;
}

void NS3Adapter::onConnectHandler() {
    ROS_INFO_STREAM("NS-3 Adapter Connected");
    cav_msgs::DriverStatus status = getStatus();
    status.status = cav_msgs::DriverStatus::OPERATIONAL;
    setStatus(status);
}

void NS3Adapter::onDisconnectHandler() {
    cav_msgs::DriverStatus status = getStatus();
    status.status = cav_msgs::DriverStatus::OFF;
    setStatus(status);
    ROS_WARN_STREAM("NS-3 Adapter Disconnected");
}

/**
* @brief Handles messages received from the NS-3 Client
*
* Populates a ROS message with the contents of the incoming OBU message, and
* publishes to the ROS 'recv' topic.
*/
void NS3Adapter::onMessageReceivedHandler(const std::vector<uint8_t> &data, uint16_t id) {
    // Create and populate the message
    auto it = std::find_if(wave_cfg_items_.begin(),wave_cfg_items_.end(),[id](const WaveConfigStruct& entry)
                                                                            {
                                                                               return entry.ns3_id == std::to_string(id);
                                                                            });

    cav_msgs::ByteArray msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "";
    msg.message_type = it != wave_cfg_items_.end() ? it->name : "Unknown";
    msg.content = data;
    // Publish it
    comms_pub_.publish(msg);

    ROS_DEBUG_STREAM("Application received Data: " << data.size() << " bytes, message: " << uint8_vector_to_hex_string(data));
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
std::vector<uint8_t> NS3Adapter::packMessage(const cav_msgs::ByteArray& message) {
    std::stringstream ss;
    auto wave_item = std::find_if(wave_cfg_items_.begin(),wave_cfg_items_.end(),[&message](const WaveConfigStruct& entry)
    {
        return entry.name == message.message_type;
    });

    WaveConfigStruct cfg;
    if(wave_item == wave_cfg_items_.end())
    {
        ROS_WARN_STREAM("No wave config entry for type: " << message.message_type << ", using defaults");
        cfg.name = message.message_type;
        cfg.channel = "CCH";  //Assuming the Default channel is not the safety related info that would be in a BSM message
        cfg.priority = "1";
        cfg.ns3_id = std::to_string((message.content[0] << 8 ) | message.content[1]);
        cfg.psid = cfg.ns3_id;
    }
    else
    {
        cfg = *wave_item;
    }

    ss << "Version=0.7" << std::endl;
    ss << "Type=" << cfg.name << std::endl;
    ss << "PSID=" << cfg.psid << std::endl;
    ss << "VehicleID=" << vehicle_id_ << std::endl;
    ss << "Priority=" << cfg.priority << std::endl;
    ss << "TxMode=ALT" << std::endl;
    ss << "TxChannel=" << cfg.channel << std::endl;
    ss << "TxInterval=0" << std::endl;
    ss << "DeliveryStart=" << std::endl;
    ss << "DeliveryStop=" << std::endl;
    ss << "Signature=False" << std::endl;
    ss << "Encryption=False" << std::endl;
    ss << "VehiclePosX=" << pose_msg_.pose.position.x << std::endl;
    ss << "VehiclePosY=" << pose_msg_.pose.position.y << std::endl;
    

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
void NS3Adapter::onOutboundMessage(const cav_msgs::ByteArrayPtr& message) {
    if(!ns3_client_.connected())
    {
        ROS_WARN_STREAM("Outbound message received but node is not connected to NS-3");
        return;
    }
    
    std::shared_ptr<std::vector<uint8_t>> message_content = std::make_shared<std::vector<uint8_t>>(std::move(packMessage(*message)));
    send_msg_queue_.push_back(std::move(message_content));
}

/**
* @brief Sends a message from the queue of outbound messages
*/
void NS3Adapter::sendMessageFromQueue() {
    if (!send_msg_queue_.empty()) {
        ROS_DEBUG_STREAM("Sending message: " << std::string(send_msg_queue_.front()->begin(),send_msg_queue_.front()->end()));
        bool success = ns3_client_.sendNS3Message(send_msg_queue_.front());
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
bool NS3Adapter::sendMessageSrv(cav_srvs::SendMessage::Request& req, cav_srvs::SendMessage::Response& res) {
    if(!ns3_client_.connected())
    {
        ROS_WARN_STREAM("Outbound message received but node is not connected to NS-3 Radio");
        res.errorStatus = 1;
        return true;
    }

    // Package data into a message shared pointer; this lets packMessage have
    // the same interface for outgoing messages from the topic and service.
    const cav_msgs::ByteArray::ConstPtr message = cav_msgs::ByteArray::ConstPtr(new cav_msgs::ByteArray(req.message_to_send));
    std::shared_ptr<std::vector<uint8_t>> message_data = std::make_shared<std::vector<uint8_t>>(std::move(packMessage(*message)));
    bool success = ns3_client_.sendNS3Message(message_data);
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

void NS3Adapter::pre_spin()
{
    // Adjust output queue size if config changed.
    if(ns3_client_error_)
    {
        ROS_ERROR_STREAM("NS-3 Client Error : " << ns3_client_error_.message());
        ns3_client_.close();
        ns3_client_error_.clear();
    }
    //If we are not connected
    //TODO: Set up functionality for disconnected NS-3
    /*if (!connecting_ && !ns3_client_.connected())
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
           // ROS_INFO("Connecting to %s:%u", cfg.dsrc_address.c_str(), cfg.dsrc_listening_port);
            ROS_INFO("Local port: %u", cfg.listening_port);
            try {
                if (!ns3_client_.connect(cfg.dsrc_address, cfg.dsrc_listening_port,
                                          cfg.listening_port, ec))
                {
                    ROS_WARN_STREAM("Failed to connect, err: " << ec.message());
                }
            }catch(std::exception e)
            {
                ROS_ERROR_STREAM("Exception connecting to dsrc radio: " << e.what() << " error_code: " << ec.message());
               // ROS_ERROR_STREAM("Config:\n\tdsrc_address:" << cfg.dsrc_address
                                         << "\n\tdsrc_listening_port:" << cfg.dsrc_listening_port
                                         << "\n\tlistening_port:" << cfg.listening_port);
            }


            connecting_ = false;
        }));
    }*/
}


void NS3Adapter::post_spin() {
    sendMessageFromQueue();
}

/*void NS3Adapter::dynReconfigCB(dsrc::DSRCConfig & cfg, uint32_t level)
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
        ns3_client_.close();
    }
}*/

void NS3Adapter::loadWaveConfig(const std::string &fileName)
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
                         "      \"ns3_id\": {\n"
                         "        \"description\": \"J2735 NS-3 id assigned to message type in decimal\",\n"
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
                         "    \"required\":[\"name\",\"psid\",\"ns3_id\",\"channel\",\"priority\"]"
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
                                     entry["ns3_id"].GetString(),
                                     entry["channel"].GetString(),
                                     entry["priority"].GetString());

    }
}

void NS3Adapter::shutdown()
{
    ROS_INFO("Shutdown signal received from DriverApplication");
    if (connect_thread_)
    {
        ROS_INFO("Cleaning up connection thread");
        connect_thread_->join();
        connect_thread_.reset();
    }

    ROS_INFO("Closing connection to radio");
    ns3_client_.close();

}

void NS3Adapter::pose_cb(geometry_msgs::PoseStamped pose_msg)
{
    pose_msg_ = pose_msg;

    /*TODO: Add Pose Functionality*/
}

cav_msgs::DriverStatus NS3Adapter::getDriverStatus()
{
    return getStatus();
}

std::deque<std::shared_ptr<std::vector<uint8_t>>> NS3Adapter::getMsgQueue()
{
    return send_msg_queue_;
}
