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
#include <geometry_msgs/TwistStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <ros/time.h>

#include <deque>
#include <map>

namespace cav
{

class ExtrapolationException : public std::runtime_error
{
public:
    explicit ExtrapolationException(const std::string &__arg) : runtime_error(__arg) 
    {

    }
}; // End ExtrapolationException class

template<typename T>
T lerp(const T& t0, const T& t1,const boost::posix_time::time_duration& dt);

template<typename T>
class LinearHistoryBuffer
{
public:
    std::deque< std::pair<boost::posix_time::ptime, T> > history_buffer;
    boost::posix_time::time_duration buffer_period_length;

    explicit LinearHistoryBuffer(boost::posix_time::time_duration length) : buffer_period_length(length)
    {

    }

    /**
     * Add an entry e, with timestamp t to the buffer
     * @param e
     * @param t
     */
    void addEntry(const T& e, const boost::posix_time::ptime& t)
    {
        if(!history_buffer.empty())
        {
            auto& back = history_buffer.back();
            if(t < back.first)
            {
                history_buffer.clear();
            }
        }

        history_buffer.push_back(std::make_pair(t, e));
        while(history_buffer.front().first < (t - buffer_period_length))
        {
            history_buffer.pop_front();
        }
    }

    /**
     * Get a linear interpolation from the buffer for time T.
     * @param t
     * @param out
     * @return false - if the buffer is empty and cant interpolate
     */
    bool getLerp(const boost::posix_time::ptime& t, T& out)
    {
        if(history_buffer.empty())
        {
            return false;
        }

        if(t < history_buffer.front().first)
        {
            throw ExtrapolationException("Lookup would require extrapolation into past");
        }

        typename std::deque<std::pair<boost::posix_time::ptime, T>>::const_iterator it = history_buffer.cbegin();

        while(it!= history_buffer.end() && it->first < t)
        {
            if(it->first == t)
            {
                out = it->second;
                return true;
            }
            it++;
        }

        if(it == history_buffer.end())
        {
            throw ExtrapolationException("Lookup would require extrapolation into future");
        }

        if(it == history_buffer.begin())
        {
            out = it->second;
        }
        else
        {
            out = lerp<T>((it - 1)->second, it->second, t - (it - 1)->first);
        }
        return true;
    }
}; // End LinearHistoryBuffer class

class TwistHistoryBuffer
{
    std::map<std::string, LinearHistoryBuffer<geometry_msgs::TwistStamped>> q_;
    ros::Duration q_duration_length_;

public:

    explicit TwistHistoryBuffer(ros::Duration length) : q_duration_length_(length)
    {

    }

    void addTwist(const geometry_msgs::TwistStampedConstPtr& twist)
    {
        auto q_it = q_.find(twist->header.frame_id);
        if(q_it == q_.end())
        {
            auto ret = q_.insert(std::make_pair(twist->header.frame_id, LinearHistoryBuffer<geometry_msgs::TwistStamped>(q_duration_length_.toBoost())));
            q_it = ret.first;
        }

        q_it->second.addEntry(*twist, twist->header.stamp.toBoost());
    }

    bool getTwist(geometry_msgs::TwistStamped& twist)
    {
        return getTwist(twist.header.frame_id, twist.header.stamp, twist);
    }

    bool getTwist(const std::string &frame_id, const ros::Time& time, geometry_msgs::TwistStamped& out_twist)
    {
        auto q_it = q_.find(frame_id);
        if(q_it == q_.end())
        {
            return false;
        }

        return q_it->second.getLerp(time.toBoost(), out_twist);
    }

    bool getLatest(const std::string &frame_id, geometry_msgs::TwistStamped& out_twist)
    {
        auto q_it = q_.find(frame_id);
        if(q_it == q_.end())
        {
            return false;
        }

        if(q_it->second.history_buffer.empty())
        {
            return false;
        }

        out_twist = q_it->second.history_buffer.back().second;
        return true;
    }
}; // End TwistHistoryBuffer class

template<>
inline geometry_msgs::TwistStamped lerp<geometry_msgs::TwistStamped>(const geometry_msgs::TwistStamped&t0, const geometry_msgs::TwistStamped&t1, const boost::posix_time::time_duration& dt)
{
    double delta_t = dt.total_microseconds() / 1000000UL;
    geometry_msgs::TwistStamped ret;
    ret.header.frame_id = t0.header.frame_id;
    ret.header.stamp = ros::Time::fromBoost(t0.header.stamp.toBoost() + dt);

    // Linear
    {
        tf2::Vector3 t0_v, t1_v, out_v;
        tf2::convert(t0.twist.linear, t0_v);
        tf2::convert(t1.twist.linear, t1_v);

        out_v = t0_v + (t1_v - t0_v) * delta_t;
        tf2::convert(out_v, ret.twist.linear);
    }
    
    // Angular
    {
        tf2::Vector3 t0_v, t1_v, out_v;
        tf2::convert(t0.twist.angular, t0_v);
        tf2::convert(t1.twist.angular, t1_v);

        out_v = t0_v + (t1_v - t0_v) * delta_t;
        tf2::convert(out_v, ret.twist.angular);
    }

    return ret;
}

} // End cav namespace
