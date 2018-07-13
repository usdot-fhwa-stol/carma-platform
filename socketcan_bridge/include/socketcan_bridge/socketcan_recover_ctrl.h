#ifndef SOCKETCAN_RECOVER_CTRL_H
#define SOCKETCAN_RECOVER_CTRL_H

#include <can_msgs/CanState.h>
#include <socketcan_interface/socketcan.h>
#include <ros/ros.h>

namespace socketcan_bridge
{

class SocketCANRecoverCtrl
{
public:

	/**
     * @brief Constructor for the recovery control class
     *
     * Initializes the publisher, and sets up a timer to periodically check the bus state
     */
    SocketCANRecoverCtrl(ros::NodeHandle* nh, ros::NodeHandle* nh_param, boost::shared_ptr<can::DriverInterface> driver);

    ~SocketCANRecoverCtrl()
    {
        timer_.stop();
    }

private:

	/**
 	* @brief Publishes the status of the bus
 	*/
    void publishStatus(const can::State & state);

    /**
     * @brief Recover the bus from an error state
     *
     * Calls the driver's recover() function
     */
    void recover();

    /**
     * @brief Checks the state of the bus, if !statie.isReady() then the
     * recover timer is started to fire in 5secs, otherwise we stop the timer
     */
    void stateCallback(const can::State & s);


    ros::Publisher state_pub_;
    boost::shared_ptr<can::DriverInterface> driver_;
    ros::WallTimer timer_;

    can::StateInterface::StateListener::Ptr state_listener_;

};

};  // namespace socketcan_bridge


#endif
