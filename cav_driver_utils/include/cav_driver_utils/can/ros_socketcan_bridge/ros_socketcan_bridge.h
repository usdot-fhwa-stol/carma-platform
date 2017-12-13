#pragma once
#include <cav_driver_utils/can/can_interface.h>

#include <can_msgs/Frame.h>

#include <ros/callback_queue.h>
#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include <ros/publisher.h>
#include <ros/spinner.h>
#include <unordered_set>

namespace cav
{

class ROSSocketCANBridge : public cav::CANInterface
{

    ros::NodeHandle nh_;
    ros::Subscriber recv_sub_;
    bool recv_ = false;
    ros::Publisher out_pub_;
    bool write_ = false;

    std::unique_ptr<ros::AsyncSpinner> spinner_;
    std::unique_ptr<ros::NodeHandle> async_nh_;
    ros::CallbackQueue async_q_;
    std::unordered_set<unsigned int> filters_;

    void recv_cb(const can_msgs::FrameConstPtr& msg);

public:

    ROSSocketCANBridge(const std::string& recv_topic, const std::string& out_topic);

    virtual ~ROSSocketCANBridge() override = default;

    virtual void close() override;

    virtual void removeFilters() override;

    virtual void setFilters(const std::vector<uint16_t> &can_ids) override;

    virtual void write(const cav::CANFrameStamped &frame) override;

    virtual bool is_open() override;

};


}