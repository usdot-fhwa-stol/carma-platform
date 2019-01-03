#pragma once
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


#include <can_msgs/Frame.h>

#include <ros/callback_queue.h>
#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include <ros/publisher.h>
#include <ros/spinner.h>

#include <boost/signals2/signal.hpp>
#include <boost/endian/conversion.hpp>

#include <thread>
#include <queue>
#include <memory>
#include <condition_variable>
#include <ostream>

namespace torc
{

class SRXDBWController  {

public:

    /**
     * @brief Class to help set PID values
     *
     * Since the hardware communicates PID values as unsigned values this class
     * allows use to verify that desired and feedback PID values match
     */
    class PIDParams_t
    {
        friend class SRXDBWController;
    public:

        PIDParams_t() : p_(0), i_(0), d_(0) { }

        /**
         * @brief Returns the unscaled proportional gain
         * @return
         */
        inline double p() const { return p_ / (double)div_; };

        /**
         * @brief Returns the unscaled integral gain
         * @return
         */
        inline double i() const { return i_ / (double)div_; };

        /**
         * @brief Returns the unscaled derivative gain
         * @return
         */
        inline double d() const { return d_ / (double)div_; };


        /**
         * @brief Sets the PID parameters using unscaled values
         *
         * The internal representation of these paramets takes the form of
         * uint16 p,i,d,div. This function determines a suitable divisor
         * to use to best represent the pid values passed and sets the internal
         * representation
         *
         * @param p proportional gain
         * @param i integral gain
         * @param d derivative gain
         */
        void setF(double p, double i, double d);

        /**
         * @brief Sets the internal representation of the p i d div variables
         *
         *  The gains are calculated by numerator/divisor for the perspective
         *  p,i,d numerators
         *
         * @param p proportional gain numerator
         * @param i integral gain numerator
         * @param d derivative gain numerator
         * @param div divisor
         */
        void setU16(uint16_t p, uint16_t i, uint16_t d, uint16_t div);

        /**
         * @brief Compares the PIDParams for equality
         * @param rhs
         * @return true if the internal representation is the same
         */
        inline bool operator==(const PIDParams_t& rhs) const
        {
            return p_ == rhs.p_ && i_ == rhs.i_ && d_ == rhs.d_ && div_ == rhs.div_;
        }

        /**
         * @brief Compares the PIDParams for inequality
         * @param rhs
         * @return true if the internal representation is not the same
         */
        inline bool operator!=(const PIDParams_t& rhs) const
        {
            return !(*this==rhs);
        }

    private:
        uint16_t p_,i_,d_,div_;

    };


    /**
     * @brief Unpacked version of the light parameters sent/received to the controller
     */
    struct LightParams_t
    {
        bool GreenFlashOn;
        bool GreenSolidOn;
        bool LeftArrowOn;
        bool RightArrowOn;
        bool TakeDownOn;
        bool FlashOn;

        bool operator==(const torc::SRXDBWController::LightParams_t& rhs);
        bool operator!=(const torc::SRXDBWController::LightParams_t& rhs);
    };


    /**
     * @brief Used to identify which light bar the light params correspond to
     */
    enum class LightID_t
    {
        Front,
        Rear
    };


    /**
     * @brief This maps to the CommandMode in the send command message (0x100) to the controller
     *
     * It is also what is reported in the throttle/brake feedback method
     */
    enum class CommandMode_t : uint8_t
    {
        Disabled = 0,
        WrenchEffort = 1,
        ClosedLoop = 2,
        Reserved = 3
    };


    /**
     * @brief Unpacked version of the throttlefeedback message (x1A0) from the controller
     */
    struct ThrottleOutputFeedback_t
    {
        bool AccelCmdActive;
        bool AccelCmdTimeout;
        CommandMode_t AccelCmdModeEcho;
        bool VehicleBusTimeout;
        bool DriverOverride;
        bool MessageInjected;
        bool AccelPairTimeout;
        double InjectedTorque;
        bool AccelDBWEnabled;
        bool ACCPrimed;
        bool ThrottleReengageReq;


    };


    /**
     * @brief Unpacked version of the brakefeedback message (x1B0) from the controller
     */
    struct BrakeOutputFeedback_t
    {

        bool BrakeCmdInjection;
        bool BrakeCmdTimeout;
        CommandMode_t BrakeCmdModeEcho;
        bool VehicleBusTimeout;
        bool OverrideEnabled;
        bool MessageInjected;
        bool BrakePairTimeout;
        double InjectedBrakeForce;
        bool BrakeDBWEnabled;

    };


    /**
     * @brief Unpacked version of the send command message (x100) sent to the controller
     */
    struct ControlMessage_t
    {
        uint8_t Counter;
        bool EnableDBW;
        CommandMode_t CommandMode;
        uint16_t WrenchEffort;
        uint16_t SpeedControl;
        uint16_t MaxAccel;
    };



    /**
     * @brief structure to store the socket can ErrorCode and produce a valid message from them
     */
    struct ErrorCode_t
    {
        unsigned int code = 0;
        const std::string what() const;
    };


private:


    //ROS members
    ros::NodeHandle& nh_;
    ros::NodeHandle& pnh_;
    ros::Subscriber can_in_sub_;
    ros::Publisher can_out_pub_, speed_out_pub_;
    std::shared_ptr<ros::AsyncSpinner> spinner_;
    std::shared_ptr<ros::NodeHandle> async_nh_;
    ros::CallbackQueue async_q_;
    ros::Timer version_timer_;


    // Process input members
    std::queue<can_msgs::FrameConstPtr> recv_q_;
    std::mutex recv_q_mutex_;
    std::condition_variable cv_;
    volatile bool running_ = false;
    std::shared_ptr<std::thread> process_thread_;
    bool allow_lights_when_disengaged_ = false;

    // Control message is maintains the state of the previous message sent
    ControlMessage_t ctrl_msg_;

    uint8_t firmware_revision_;



public:

    /**
     * @brief Constructor stores the nodehandles passed
     * @param nh
     * @param pnh
     */
    SRXDBWController(ros::NodeHandle& nh, ros::NodeHandle& pnh);

    virtual ~SRXDBWController();

    /**
     * @brief Connects this controller to the recv_topics and out_topics,
     *
     * @param recv_topic can_msgs/Frame topic with received frames from the SRX DBW module
     * @param out_topic can_msgs/Frame topic with outgoing frames bound for the SRX DBW module
     */
    void connect(const std::string &recv_topic, const std::string &out_topic);

    /**
     * @brief Stops listening to topics and cancels processing of queue
     */
    void close();

    /**
     * @brief Sends a disable robotic command to the SRX DBW module
     */
    void disableRoboticControl();

    /**
     * @brief Sends a wrench effort command to the SRX DBW Module
     * @param effort [-100,100] percent effort, values out side of range are bounded to extremes
     */
    void setWrenchEffort(float effort);

    /**
     * @brief Sends a closed loop control command to the SRX DBW Module
     * @param speed - set speed m/s
     * @param max_accel - max accel to use ( bounds the PID controller ) m/s^2
     */
    void setSpeedAccel(double speed, double max_accel);


    /**
     * @brief Sends a set light command to the SRX DBW Module
     * @param id - id of the lights to set
     * @param params - light parameters to be sent
     */
    void setLights(const LightID_t &id, const LightParams_t &params);

    /**
     * @brief Sends a set PID command to the SRX DBW Module
     * @param params - PID params to send
     */
    void setPID(PIDParams_t &params);

    /**
     * @brief Sends a request to the controller to send firmware version
     */
    void sendFirmwareVersionRequest();

    /**
     * @brief Signals that the SRXDBWController is connected to topics and processing inputs
     *
     * Is called from the process thread
     */
    boost::signals2::signal<void ()> onConnect;

    /**
     * @brief Signals that the SRXDBWController is no longer processing inputs or sending commands
     *
     * Is called from the process thread
     */
    boost::signals2::signal<void ()> onClose;

    /**
     * @brief Signal on receipt of process Light status message received
     *
     * Is called from the process thread
     */
    boost::signals2::signal<void (LightID_t, LightParams_t)> onLightStatusRecv;


    /**
     * @brief Signal on receipt of process Throttle feedback message received
     *
     * Is called from the process thread
     */
    boost::signals2::signal<void (ThrottleOutputFeedback_t)> onThrottleFeedbackRecv;


    /**
     * @brief Signal on receipt of process Brake Feedback message received
     *
     * Is called from the process thread
     */
    boost::signals2::signal<void (BrakeOutputFeedback_t)> onBrakeFeedbackRecv;


    /**
     * @brief Signal on receipt of process PID echo message received
     *
     * Is called from the process thread
     */
    boost::signals2::signal<void (const PIDParams_t&)> onPIDEchoRecv;


    /**
     * @brief fired on received ErrorFrame on received topic
     *
     * Is called from the process thread
     */
    boost::signals2::signal<void (const ErrorCode_t&)> onErrorFrame;


    /**
     * @brief fired on send command
     *
     * Is called from the sending command thread
     *
     */
    boost::signals2::signal<void (const ControlMessage_t&)> onSentCommand;

private:


    /**
     * @brief Packs and sends the ctrl_msg_ out to the sent message topic
     */
    void sendSpeedCmd();


    /**
     * @brief Callback for receive message topic
     *
     * Pushes received messages to the process thread to be queued for process
     * @param msg
     */
    void frameReceivedCB(const can_msgs::FrameConstPtr& msg);


    /**
     * @brief process thread function
     *
     * This method monitors the recv_q_ for updates and unpacks messages to their data types and calls the corresponding
     * events.
     */
    void process();


    /**
     * @brief Packs a param message into bytes (0x101)
     * @param params
     * @param out
     */
    static void packPIDParams(const PIDParams_t &params, std::vector<uint8_t> &out);

    /**
     * @brief packs a Control Message into bytes (0x100)
     * @param msg
     * @param out
     */
    static void packSpeedControlMessage(const ControlMessage_t &msg, std::vector<uint8_t> &out);


    /**
     * @brief Unpacks throttle feedback message (0x1A0)
     * @param msg
     * @param out
     * @return true if succesfully unpacked, false otherwise
     */
    static bool unpackThrottleOutputFeedback(const std::vector<uint8_t> &msg, ThrottleOutputFeedback_t &out);


    /**
     * @brief Unpacks brake feedback message (0x1B0)
     * @param msg
     * @param out
     * @return true if succesfully unpacked, false otherwise
     */
    static bool unpackBrakeOutputFeedback(const std::vector<uint8_t> &msg, BrakeOutputFeedback_t &out);


    /**
     * @brief Unpacks PID message (0x111)
     * @param msg
     * @param out
     * @return true if succesfully unpacked, false otherwise
     */
    static bool unpackPIDMessage(const std::vector<uint8_t> &msg, PIDParams_t &out, const boost::endian::order& order);


    /**
     * @brief Unpacks LightState message (0x113 | 0x114)
     * @param msg
     * @param out
     * @return true if succesfully unpacked, false otherwise
     */
    static bool unpackLightState(const std::vector<uint8_t> &msg, LightParams_t &out);


};


}

//Helper methods to convert data types to string

std::ostream &operator<<(std::ostream & str,const torc::SRXDBWController::LightParams_t& obj);
std::ostream &operator<<(std::ostream& str, const torc::SRXDBWController::CommandMode_t& mode);
std::ostream &operator<<(std::ostream & str, const torc::SRXDBWController::ThrottleOutputFeedback_t& obj);
std::ostream &operator<<(std::ostream & str, const torc::SRXDBWController::BrakeOutputFeedback_t& obj);
std::ostream &operator<<(std::ostream& str, const torc::SRXDBWController::PIDParams_t& params);
std::ostream &operator<<(std::ostream & str, const torc::SRXDBWController::ControlMessage_t& msg);

