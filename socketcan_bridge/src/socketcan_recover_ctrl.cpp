#include <socketcan_bridge/socketcan_recover_ctrl.h>

namespace socketcan_bridge
{
    
    /**
     * @brief Constructor for the recovery control class
     *
     * Initializes the publisher, and sets up a timer to periodically check the bus state
     */
    SocketCANRecoverCtrl::SocketCANRecoverCtrl(ros::NodeHandle* nh, ros::NodeHandle* nh_param,
        boost::shared_ptr<can::DriverInterface> driver) : driver_(driver)
    {
        state_pub_ = nh->advertise<can_msgs::CanState>("can_state", 1, true);

        timer_ = nh->createWallTimer(ros::WallDuration(5), boost::bind(&SocketCANRecoverCtrl::recover, this), true, false);

        state_listener_ = driver_->createStateListener(
            can::StateInterface::StateDelegate(this, &SocketCANRecoverCtrl::stateCallback));

    }

    /**
     * @brief Checks the state of the bus, if !statie.isReady() then the
     * recover timer is started to fire in 5secs, otherwise we stop the timer
     */
    void SocketCANRecoverCtrl::stateCallback(const can::State & state) {
        publishStatus(state);
        if(!state.isReady() && state.driver_state != can::State::closed)
        {            
            timer_.start();
        }
        else
        {
            timer_.stop();
        }
    }

    /**
     * @brief Recover the bus from an error state
     *
     * Calls the driver's recover() function
     */
    void SocketCANRecoverCtrl::recover() 
    {
        timer_.stop();
        if(driver_->recover()) {
            ROS_INFO("CAN driver timed out, successfully recovered");
        } 
        else 
        {
            ROS_WARN("CAN driver timed out, recovery failed");
            timer_.start();
        }
    }

    /**
     * @brief Publishes the status of the bus
     */
    void SocketCANRecoverCtrl::publishStatus(const can::State & state) {
        can_msgs::CanState state_msg;
        switch(state.driver_state) {
            case can::State::open:
                state_msg.driver_state = can_msgs::CanState::OPEN;
                break;
            case can::State::closed:
                state_msg.driver_state = can_msgs::CanState::CLOSED;
                break;
            case can::State::ready:
                state_msg.driver_state = can_msgs::CanState::READY;
                break;
            default:
                state_msg.driver_state = can_msgs::CanState::CLOSED;
                break;
        }

        state_pub_.publish(state_msg);
    }


};  // namespace socketcan_bridge
