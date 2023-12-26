#pragma once

#include <rclcpp/rclcpp.hpp>
#include <autoware_msgs/msg/control_command_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <hardcoded_params/control_limits/control_limits.h>

namespace twist_filter{

constexpr double MAX_LONGITUDINAL_ACCEL_HARDCODED_LIMIT_M_S_2 = hardcoded_params::control_limits::MAX_LONGITUDINAL_ACCEL_MPS2;

class LongitudinalAccelLimiter
{
    public: 
        /**
         * Default constructor, not recommended for use.
         */
        LongitudinalAccelLimiter():
            _accel_limit(3.5),
            _initialized(false),
            _prev_v(0.0),
            _prev_t(0.0) {};


        /**
         * \brief Constructor for LongitudinalAccelLimiter
         * 
         * \param accel_limit The acceleration limit that should be enforced, in
         * units of m/s/s
         */
        LongitudinalAccelLimiter(double accel_limit):
            _accel_limit(accel_limit),
            _initialized(false),
            _prev_v(0.0),
            _prev_t(0.0) {};  

        /**
         * Limit the longitudinal acceleration found in the input ControlCommandStamped
         * 
         * \param msg The message to be evaluated
         * \return A copy of the message with the longitudinal accel limited 
         * based on params or hardcoded limit
         */
        autoware_msgs::msg::ControlCommandStamped
            longitudinalAccelLimitCtrl(const autoware_msgs::msg::ControlCommandStamped& msg);

                /**
         * Limit the longitudinal acceleration found in the input TwistStamped
         * 
         * \param msg The message to be evaluated
         * \return A copy of the message with the longitudinal accel limited 
         * based on params or hardcoded limit
         */
        geometry_msgs::msg::TwistStamped
            longitudinalAccelLimitTwist(const geometry_msgs::msg::TwistStamped& msg);

    private:
        double _accel_limit;
        bool _initialized;
        double _prev_v;
        rclcpp::Time _prev_t; 
};

}