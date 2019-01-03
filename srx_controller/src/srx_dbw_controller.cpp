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

#include <srx_controller/srx_dbw_controller.h>
#include <boost/math/special_functions/round.hpp>
#include <boost/endian/conversion.hpp>

#include <cmath>
#include <sstream>

torc::SRXDBWController::SRXDBWController(ros::NodeHandle &nh, ros::NodeHandle &pnh) : nh_(nh), pnh_(pnh), ctrl_msg_{}, firmware_revision_(0)
{

}

torc::SRXDBWController::~SRXDBWController() {
    close();
}

void torc::SRXDBWController::connect(const std::string &recv_topic, const std::string &out_topic) {
    if(running_) return;

    // Check if we will allow the light bar to remain on when robotic is off
    pnh_.param<bool>("allow_lights_when_disengaged", allow_lights_when_disengaged_, false);

    //We use the async nodehandle to connect to the received topic so that we can receive can messages
    //regardless of the speed of the Driver Application that is using this
    async_nh_.reset(new ros::NodeHandle(nh_.getNamespace()));
    async_nh_->setCallbackQueue(&async_q_);
    version_timer_ = async_nh_->createTimer(ros::Duration(5.0),[this](const ros::TimerEvent&){ sendFirmwareVersionRequest(); });
    can_in_sub_ = async_nh_->subscribe<can_msgs::Frame>(recv_topic,10, [this](const can_msgs::FrameConstPtr& msg){frameReceivedCB(msg);});

    //Using two publishers because we want to limit the publish queue of the speed messages by 1, but it is ok to queue
    //the other messages sent
    can_out_pub_ = nh_.advertise<can_msgs::Frame>(out_topic,10);
    speed_out_pub_ = nh_.advertise<can_msgs::Frame>(out_topic, 1);

    spinner_.reset(new ros::AsyncSpinner(1,&async_q_));
    spinner_->start();

    running_ = true;
    process_thread_.reset(new std::thread([this](){process();}));

    //Calls the onConnect() signal. This signals to the connected caller that this driver has finished its initialization
    //setup and is listening for incoming messages and ready to send out going messages
    onConnect();

}

void torc::SRXDBWController::close() {
    if(!running_) return;
    running_ = false;

    if (!allow_lights_when_disengaged_) {
        // Turn off lights
        LightParams_t lightsOff{};
        setLights(LightID_t::Front, lightsOff);
        setLights(LightID_t::Rear, lightsOff);
    }

    // Shutdown threads
    can_in_sub_.shutdown();
    cv_.notify_all();
    process_thread_->join();
    spinner_->stop();
    onClose();
}

void torc::SRXDBWController::disableRoboticControl() {
    ctrl_msg_.CommandMode = CommandMode_t::Disabled;
    ctrl_msg_.EnableDBW = false;
    sendSpeedCmd();

    if (!allow_lights_when_disengaged_) {
        // Turn off lights
        LightParams_t lightsOff{};
        setLights(LightID_t::Front, lightsOff);
        setLights(LightID_t::Rear, lightsOff);
    }
}

void torc::SRXDBWController::setWrenchEffort(float effort) {
    if(!running_) return;

    //Bound effort between [-100,100]
    if(effort < -100.0) effort = -100.0f;
    else if(effort > 100.0) effort = 100.0f;

    ctrl_msg_.CommandMode = CommandMode_t::WrenchEffort;
    ctrl_msg_.EnableDBW = true;

    //If we are about 0 set to, output effort is 0-8000 where real effort = (output - 4000)/40.0
    if(std::fabs(effort) < 0.025)
    {
        ctrl_msg_.WrenchEffort = 4000; //4000 is stop
    }
    else
    {
        int tmp = boost::math::iround(4000*effort/100.0f);
        tmp += 4000;
        if(tmp < 0) tmp = 0;
        else if(tmp > 8000) tmp = 8000;
        ctrl_msg_.WrenchEffort = (uint16_t)(tmp);
    }

    sendSpeedCmd();
}

void torc::SRXDBWController::setLights(const torc::SRXDBWController::LightID_t &id,
                                       const torc::SRXDBWController::LightParams_t &params) {
    if(!running_) return;

    can_msgs::Frame fr;
    switch (id)
    {
        case LightID_t::Front:
            fr.id = 0x103;
            break;
        case LightID_t::Rear:
            fr.id = 0x104;
            break;
    }

    fr.header.stamp = ros::Time::now();
    fr.header.frame_id = "";

    fr.dlc  = 1;
    fr.data[0] = 0;

    fr.data[0] |= params.TakeDownOn ? 1 : 0;
    fr.data[0] <<= 1;

    fr.data[0] |= params.FlashOn ? 1 : 0;
    fr.data[0] <<= 1;

    fr.data[0] |= params.GreenSolidOn ? 1 : 0;
    fr.data[0] <<= 1;

    fr.data[0] |= params.RightArrowOn ? 1 : 0;
    fr.data[0] <<= 1;

    fr.data[0] |= params.LeftArrowOn ? 1 : 0;
    fr.data[0] <<= 1;

    fr.data[0] |= params.GreenFlashOn ? 1 : 0;

    can_out_pub_.publish(fr);

}

void torc::SRXDBWController::setSpeedAccel(double speed, double max_accel) {
    if(!running_) return;
    if(speed < 0 || max_accel < 0) return;

    ctrl_msg_.CommandMode = CommandMode_t::ClosedLoop;
    ctrl_msg_.EnableDBW = true;
    double kmh = speed*3600.0/1000.0; //convert to km/h
    double accel_kmhs = max_accel*3600/1000.0; //convert to km/(h*s)

    //The CAN spec uses a scaling factor of 0.03125
    ctrl_msg_.SpeedControl = (uint16_t) boost::math::iround(kmh/0.03125);
    ctrl_msg_.MaxAccel = (uint16_t) boost::math::iround(accel_kmhs/0.03125);

    sendSpeedCmd();
}

void torc::SRXDBWController::setPID(torc::SRXDBWController::PIDParams_t &params) {
    can_msgs::Frame fr;

    fr.id = 0x101;
    fr.header.stamp = ros::Time::now();
    fr.header.frame_id = "";

    std::vector<uint8_t> data;
    packPIDParams(params, data);

    fr.dlc = (unsigned char)data.size();

    std::copy(data.begin(),data.end(),fr.data.begin());

    can_out_pub_.publish(fr);
}


void torc::SRXDBWController::sendFirmwareVersionRequest()
{
    can_msgs::Frame fr;
    fr.id = 0x105;
    fr.header.stamp = ros::Time::now();
    fr.header.frame_id = "";

    fr.dlc = 0;
    can_out_pub_.publish(fr);

}

void torc::SRXDBWController::sendSpeedCmd() {
    if(!running_) return;

    can_msgs::Frame fr;

    fr.id = 0x100;
    fr.header.stamp = ros::Time::now();
    fr.header.frame_id = "";

    std::vector<uint8_t> data;

    ctrl_msg_.Counter++;
    ctrl_msg_.Counter &= 0x03;
    packSpeedControlMessage(ctrl_msg_, data);

    fr.dlc = (unsigned char)data.size();

    std::copy(data.begin(),data.end(),fr.data.begin());

    speed_out_pub_.publish(fr);
    onSentCommand(ctrl_msg_);
}

void torc::SRXDBWController::frameReceivedCB(const can_msgs::FrameConstPtr &msg) {
    if(!running_) return;
    {
        std::unique_lock<std::mutex> lock(recv_q_mutex_);
        recv_q_.push(msg);
    }

    cv_.notify_all();
}

void torc::SRXDBWController::process() {
    while(running_)
    {
        can_msgs::FrameConstPtr entry;
        {
            std::unique_lock<std::mutex> lock(recv_q_mutex_);
            if(recv_q_.empty())
            {
                cv_.wait(lock,[this](){return !recv_q_.empty() || !running_;});
            }

            if(!running_) break;
            entry = recv_q_.front();
            recv_q_.pop();

        }

        //If this entry is an error frame process it
        if(entry->is_error != 0u)
        {
            ErrorCode_t e;
            e.code = entry->id;
            onErrorFrame(e);
        }
        else
        {
            std::vector<uint8_t> data(entry->data.begin(),entry->data.end());
            switch(entry->id)
            {
                case 0x1A0: //Throttle Output Feedback
                {
                    ThrottleOutputFeedback_t feedback{};
                    if(unpackThrottleOutputFeedback(data, feedback))
                        onThrottleFeedbackRecv(feedback);
                    break;
                }
                case 0x111: //PID Echo
                {
                    PIDParams_t params{};
                    if(unpackPIDMessage(data, params, firmware_revision_ < 3 ? boost::endian::order::little : boost::endian::order::big))
                        onPIDEchoRecv(params);
                    break;
                }
                case 0x113: //Front Light States
                {
                    LightParams_t params{};
                    if(unpackLightState(data, params))
                        onLightStatusRecv(LightID_t::Front, params);
                    break;
                }
                case 0x114: //Rear Light States
                {
                    LightParams_t params{};
                    if(unpackLightState(data, params))
                        onLightStatusRecv(LightID_t::Rear, params);
                    break;
                }
                case 0x1B0: //Brake Actuator Position Feedback
                {
                    BrakeOutputFeedback_t feedback{};
                    if(unpackBrakeOutputFeedback(data, feedback))
                        onBrakeFeedbackRecv(feedback);
                    break;
                }
                case 0x115:
                {
                    firmware_revision_ = data[1];
                    break;
                }
                default:
                    break;
            }

        }


    }

}

void torc::SRXDBWController::packPIDParams(const PIDParams_t &params, std::vector<uint8_t> &out) {
    out.resize(8);
    uint16_t tmp;
    tmp = boost::endian::native_to_big(params.p_);
    memcpy(&out[0], &tmp, 2);

    //Integral Gain
    tmp = boost::endian::native_to_big(params.i_);
    memcpy(&out[2], &tmp, 2);


    //Differential Gain
    tmp = boost::endian::native_to_big(params.d_);
    memcpy(&out[4], &tmp, 2);

    //Divisor
    tmp = boost::endian::native_to_big(params.div_);
    memcpy(&out[6], &tmp, 2);

}

void
torc::SRXDBWController::packSpeedControlMessage(const ControlMessage_t &msg, std::vector<uint8_t> &out) {
    out.resize(8);

    out[0] = (uint8_t)(msg.Counter & 0x03);

    if(msg.EnableDBW)
        out[0] |= 0x4;

    out[0] |= (static_cast<uint8_t>(msg.CommandMode) << 3);

    uint16_t tmp = msg.WrenchEffort;
    tmp = boost::endian::native_to_big(tmp);
    memcpy(&out[1], &tmp, 2);

    tmp = msg.SpeedControl;
    tmp = boost::endian::native_to_big(tmp);
    memcpy(&out[3], &tmp, 2);

    tmp = msg.MaxAccel;
    tmp = boost::endian::native_to_big(tmp);
    memcpy(&out[5], &tmp, 2);

    //Calculate checksum
    out[7]  = 0;
    for(int i = 0; i < 7; i++)
    {
        out[7] += out[i];
    }

    out[7] = ~out[7] + (unsigned char)0x01;

}

bool torc::SRXDBWController::unpackThrottleOutputFeedback(const std::vector<uint8_t> &msg,
                                                          torc::SRXDBWController::ThrottleOutputFeedback_t &out) {
    if(msg.size() < 5)
    {
        return false;
    }

    out.AccelCmdActive = ( msg[0] & 0x01 ) == 0x01;
    out.AccelCmdTimeout = (msg[0] & 0x02 ) == 0x02;
    out.AccelCmdModeEcho = static_cast<CommandMode_t>((msg[0] & 0x0C ) >> 2);
    out.VehicleBusTimeout = (msg[0] & 0x10) == 0x10;
    out.DriverOverride = (msg[0] & 0x20) == 0x20;
    out.MessageInjected = (msg[0]&0x40) == 0x40;
    out.AccelPairTimeout = (msg[0]&0x80) == 0x80;

    //Injected torque
    uint32_t torque = 0;

    memcpy(&torque,&msg[1],3);

    torque = boost::endian::big_to_native(torque);

    //Torque is sent as a 24 bit unsigned int
    torque = torque >> 8;

    //Remove scaling
    out.InjectedTorque = (torque/8.0) - 22534.0;

    out.AccelDBWEnabled = (msg[4] & 0x01) == 0x01;
    out.ACCPrimed = (msg[4] & 0x02) == 0x02;
    out.ThrottleReengageReq = (msg[4]&0x04) == 0x04;

    return true;
}

bool torc::SRXDBWController::unpackPIDMessage(const std::vector<uint8_t> &msg, torc::SRXDBWController::PIDParams_t &out, const boost::endian::order& order) {
    if(msg.size() < 8)
    {
        return false;
    }


    auto f = (order == boost::endian::order::little) ? boost::endian::little_to_native<uint16_t> : boost::endian::big_to_native<uint16_t>;

    uint16_t p,i,d,div;

    memcpy(&p,&msg[0],2);
    p = f(p);

    memcpy(&i,&msg[2],2);
    i = f(i);

    memcpy(&d,&msg[4],2);
    d = f(d);

    memcpy(&div,&msg[6],2);
    div = f(div);

    out.setU16(p, i, d, div);

    return true;
}

bool torc::SRXDBWController::unpackLightState(const std::vector<uint8_t> &msg, torc::SRXDBWController::LightParams_t &out) {
    if(msg.empty())
        return false;

    out.LeftArrowOn     = ( msg[0] & 0x02 ) == 0x02;
    out.RightArrowOn    = ( msg[0] & 0x04 ) == 0x04;
    out.TakeDownOn      = ( msg[0] & 0x08 ) == 0x08;
    out.FlashOn         = ( msg[0] & 0x10 ) == 0x10;

    return true;
}

bool torc::SRXDBWController::unpackBrakeOutputFeedback(const std::vector<uint8_t> &msg,
                                                       torc::SRXDBWController::BrakeOutputFeedback_t &out) {
    if(msg.size() < 4)
    {
        return false;
    }

    out.BrakeCmdInjection   = ( msg[0] & 0x01 ) == 0x01;
    out.BrakeCmdTimeout     = (msg[0] & 0x02 ) == 0x02;
    out.BrakeCmdModeEcho    = static_cast<CommandMode_t>((msg[0] & 0x0C ) >> 2);
    out.VehicleBusTimeout   = (msg[0] & 0x10) == 0x10;
    out.OverrideEnabled     = (msg[0] & 0x20) == 0x20;
    out.MessageInjected     = (msg[0] & 0x40) == 0x40;
    out.BrakePairTimeout    = (msg[0] & 0x80) == 0x80;

    uint16_t brakeForce;
    memcpy(&brakeForce,&msg[1],2);
    brakeForce = boost::endian::big_to_native(brakeForce);

    //Brake force is a signed integer of the form (val*100.0f) & 0x0FFF
    //To maintain the proper sign we first shift left 4 bits and then divide by 16
    //we then remove the 100x factor
    out.InjectedBrakeForce  = (static_cast<int>(brakeForce<<4)/16.0) * 0.01;
    out.BrakeDBWEnabled     = (msg[3]&0x01) == 0x01;

    return true;

}

std::ostream &operator<<(std::ostream & str,const torc::SRXDBWController::LightParams_t& obj)
{
    return str << "\tLeftArrowOn : " << obj.LeftArrowOn << std::endl <<
               "\tRightArrowOn : " << obj.RightArrowOn<< std::endl <<
               "\tTakeDownOn : " << obj.TakeDownOn << std::endl <<
               "\tFlashOn : " << obj.FlashOn << std::endl;
}

std::ostream &operator<<(std::ostream& str, const torc::SRXDBWController::CommandMode_t& mode)
{
    switch (mode)
    {
        case torc::SRXDBWController::CommandMode_t::WrenchEffort:
            return str << "WrenchEffort";
        case torc::SRXDBWController::CommandMode_t::ClosedLoop:
            return str << "ClosedLoop";
        case torc::SRXDBWController::CommandMode_t::Disabled:
            return str << "Disabled";
        default:
            return str << "Unknown";
    }
}

std::ostream &operator<<(std::ostream& str, const torc::SRXDBWController::PIDParams_t& params)
{
    return str << "\tp: " << params.p() << std::endl <<
                  "\ti: " << params.i() << std::endl <<
                  "\td: " << params.d() << std::endl;
}

std::ostream &operator<<(std::ostream & str, const torc::SRXDBWController::ThrottleOutputFeedback_t& obj)
{
    return str << "ThrottleOutputFeedback: " << std::endl <<
               "    AccelCmdActive " << obj.AccelCmdActive  << std::endl <<
               "    AccelCmdTimeout " << obj.AccelCmdTimeout  << std::endl <<
               "    AccelCmdModeEcho" << obj.AccelCmdModeEcho << std::endl <<
               "    VehicleBusTimeout" << obj.VehicleBusTimeout << std::endl <<
               "    DriverOverride " << obj.DriverOverride  << std::endl <<
               "    MessageInjected" << obj.MessageInjected << std::endl <<
               "    AccelPairTimeout" << obj.AccelPairTimeout << std::endl <<
               "    InjectedTorque" << obj.InjectedTorque << std::endl <<
               "    AccelDBWEnabled" << obj.AccelDBWEnabled << std::endl <<
               "    ACCPrimed" << obj.ACCPrimed << std::endl <<
               "    ThrottleReengageReq" << obj.ThrottleReengageReq << std::endl;
}

std::ostream &operator<<(std::ostream & str, const torc::SRXDBWController::BrakeOutputFeedback_t& obj)
{
    return str << "BrakeOutputFeedback: " << std::endl <<
               "    BrakeCmdInjection " << obj.BrakeCmdInjection  << std::endl <<
               "    BrakeCmdTimeout " << obj.BrakeCmdTimeout  << std::endl <<
               "    BrakeCmdModeEcho" << obj.BrakeCmdModeEcho << std::endl <<
               "    VehicleBusTimeout" << obj.VehicleBusTimeout << std::endl <<
               "    OverrideEnabled " << obj.OverrideEnabled  << std::endl <<
               "    MessageInjected" << obj.MessageInjected << std::endl <<
               "    BrakePairTimeout" << obj.BrakePairTimeout << std::endl <<
               "    InjectedBrakeForce" << obj.InjectedBrakeForce << std::endl <<
               "    BrakeDBWEnabled" << obj.BrakeDBWEnabled << std::endl;
}

std::ostream &operator<<(std::ostream & str, const torc::SRXDBWController::ControlMessage_t& msg)
{
    return str << "Command: " << std::endl <<
               "    CommandMode_t: "  << msg.CommandMode   << std::endl <<
               "    EnableDBW: "    << msg.EnableDBW     << std::endl <<
               "    WrenchEffort: " << msg.WrenchEffort  << std::endl <<
               "    SpeedControl: " << msg.SpeedControl  << std::endl <<
               "    MaxAccel: "     << msg.MaxAccel      << std::endl;

}

const std::string torc::SRXDBWController::ErrorCode_t::what() const {
    std::stringstream ss;

    if(code & 0x1)
        ss << "tx timeout|";
    if(code & 0x2)
        ss << "lost arbitration|";
    if(code & 0x4)
        ss << "controller problems|";
    if(code & 0x8)
        ss << "protocol violation|";
    if(code & 0x10)
        ss << "transceiver error|";
    if(code & 0x20)
        ss << "no ack received|";
    if(code & 0x40)
        ss << "bus off|";
    if(code & 0x80)
        ss << "bus error|";
    if(code & 0x100)
        ss << "controller restarted|";


    std::string ret(ss.str());
    ret.pop_back();
    return ret;
}

void torc::SRXDBWController::PIDParams_t::setF(double p, double i, double d) {
    double max = std::max({p,i, d});
    if(max != 0.0)
    {
        int max_div =  boost::math::iround(UINT16_MAX/max);
        div_ = (uint16_t) std::min<int>(max_div, UINT16_MAX);
    }
    else
    {
        div_ = UINT16_MAX;
    }

    p_ = (uint16_t) boost::math::iround(p * div_);
    i_ = (uint16_t) boost::math::iround(i * div_);
    d_ = (uint16_t) boost::math::iround(d * div_);
}

void torc::SRXDBWController::PIDParams_t::setU16(uint16_t p, uint16_t i, uint16_t d, uint16_t div) {
    p_ = p;
    i_ = i;
    d_ = d;
    div_ = div;
}


bool torc::SRXDBWController::LightParams_t::operator!=(const torc::SRXDBWController::LightParams_t &rhs) {
    return !(*this == rhs);
}


bool torc::SRXDBWController::LightParams_t::operator==(const torc::SRXDBWController::LightParams_t &rhs) {
    return TakeDownOn   == rhs.TakeDownOn   &&
           FlashOn      == rhs.FlashOn      &&
           LeftArrowOn  == rhs.LeftArrowOn  &&
           RightArrowOn == rhs.RightArrowOn &&
           GreenFlashOn == rhs.GreenFlashOn &&
           GreenSolidOn == rhs.GreenSolidOn;
}
