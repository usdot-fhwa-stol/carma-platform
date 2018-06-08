#pragma once
#include <cav_driver_utils/can/can_interface.h>
#include <iostream>
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

    ROSSocketCANBridge(const std::string &recv_topic, const std::string &out_topic);

    virtual ~ROSSocketCANBridge() override = default;

    /**
     * @brief Closes the interface to the device
     */
    virtual void close();

    /**
     * @brief removes all CAN filters
     */
    virtual void removeFilters();

    /**
     * @brief Sets CAN filters on passed IDs. This will cause only the IDs specified to signal
     * an onFrameReceived event. Sets the can_filter on the socketcan driver
     * @param can_ids
     */
    virtual void setFilters(const std::vector<uint32_t> &can_ids);

    /**
     * @brief writes the frame out to the can bus
     * @param frame
     */
    virtual void write(const CANFrameStamped& frame);

    /**
     * @brief returns true if device is open
     * @return
     */
    virtual inline bool is_open();

};


}