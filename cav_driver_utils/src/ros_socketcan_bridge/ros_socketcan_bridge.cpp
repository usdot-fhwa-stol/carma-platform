#include <cav_driver_utils/can/ros_socketcan_bridge/ros_socketcan_bridge.h>

namespace cav
{

can_msgs::Frame toROSFrame(const CANFrameStamped& msg)
{
    can_msgs::Frame fr;
    fr.header.stamp = ros::Time::fromBoost(msg.stamp);
    fr.header.frame_id = "0";

    fr.id = msg.id;
    fr.dlc = msg.dlc;
    fr.is_extended = static_cast<unsigned char>(msg.is_extended);
    fr.is_rtr = static_cast<unsigned char>(msg.is_rtr);
    fr.is_error = static_cast<unsigned char>(msg.is_error);

    std::copy(msg.data.begin(),msg.data.end(),fr.data.begin());

}

CANFrameStamped fromROSFrame(const can_msgs::Frame& msg)
{
    CANFrameStamped fr;

    fr.stamp = msg.header.stamp.toBoost();
    fr.id = msg.id;
    fr.dlc = msg.dlc;
    fr.is_extended = msg.is_extended;
    fr.is_rtr = msg.is_rtr;
    fr.is_error = msg.is_error;

    std::copy(msg.data.begin(),msg.data.end(),fr.data.begin());
}

void ROSSocketCANBridge::recv_cb(const can_msgs::FrameConstPtr &msg) {
    if(filters_.empty() || filters_.count(msg->id) != 0)
    {
        std::shared_ptr<CANFrameStamped> data(new CANFrameStamped(fromROSFrame(*msg)));
        if(data->is_error)
        {
            onErrorFrameReceived(data);
        }
        else
        {
            onFrameReceived(data);
        }
    }
}

ROSSocketCANBridge::ROSSocketCANBridge(const std::string &recv_topic, const std::string &out_topic)
{
    async_nh_.reset(new ros::NodeHandle(nh_.getNamespace()));
    async_nh_->setCallbackQueue(&async_q_);
    if(!recv_topic.empty())
    {
        recv_sub_ = async_nh_->subscribe<can_msgs::Frame>(recv_topic,1000,&ROSSocketCANBridge::recv_cb, this);
    }

    if(!out_topic.empty())
    {
        out_pub_ = nh_.advertise<can_msgs::Frame>(out_topic,1000);
    }

    spinner_.reset(new ros::AsyncSpinner(1,&async_q_));
    spinner_->start();
}

void ROSSocketCANBridge::setFilters(const std::vector<uint16_t> &can_ids) {
    filters_.clear();
    filters_.reserve(can_ids.size());
    for(auto &it : can_ids)
    {
        filters_.insert(it);
    }
}

void ROSSocketCANBridge::removeFilters() {
    filters_.clear();
}

void ROSSocketCANBridge::close() {
    async_nh_.reset(nullptr);
}

void ROSSocketCANBridge::write(const CANFrameStamped &frame) {
    if(write_)
        out_pub_.publish(toROSFrame(frame));
}

bool ROSSocketCANBridge::is_open() {
    return async_nh_ != nullptr;
}

}