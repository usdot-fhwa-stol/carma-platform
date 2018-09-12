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

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <j2735_msgs/BSM.h>
#include <j2735_msgs/SPAT.h>
#include <j2735_msgs/MapData.h>
#include <cav_msgs/SystemAlert.h>
#include <cav_msgs/BSM.h>
#include <cav_msgs/SPAT.h>
#include <cav_msgs/MapData.h>

/**
 * @class J2735Convertor
 * @brief Is the class responsible for the ROS J2735Convertor node
 */
class J2735Convertor 
{
private:

public:
  /**
   * @brief constructor
   * @param argc - command line argument count
   * @param argv - command line arguments
   */
  J2735Convertor(int argc, char** argv);

  ~J2735Convertor() 
  {
    //TODO
  }

  int run();

private:

  //ROS
  ros::Publisher converted_bsm_pub_, converted_spat_pub_, converted_map_pub_, system_alert_pub_;
  ros::Subscriber j2735_bsm_sub_, j2735_spat_sub_, j2735_map_sub_, system_alert_sub_;
  ros::NodeHandle default_nh_;
  ros::NodeHandle bsm_nh_;
  ros::NodeHandle spat_nh_;
  ros::NodeHandle map_nh_;
  ros::CallbackQueue bsm_queue_;
  ros::CallbackQueue spat_queue_;
  ros::CallbackQueue map_queue_;


  /**
   * TODO
   */
  void initialize();

  /**
   * TODO
   * @brief Handles outbound messages from the ROS network
   * @param message
   *
   * This method packs the message according to the J2375 2016 standard,
   * and sends it to the client program
   */
  void j2735BsmHandler(const j2735_msgs::BSMConstPtr& message);

  /**
   * TODO
   * @brief Handles outbound messages from the ROS network
   * @param message
   *
   * This method packs the message according to the J2375 2016 standard,
   * and sends it to the client program
   */
  void j2735SpatHandler(const j2735_msgs::SPATConstPtr& message);

  /**
   * TODO
   * @brief Handles outbound messages from the ROS network
   * @param message
   *
   * This method packs the message according to the J2375 2016 standard,
   * and sends it to the client program
   */
  void j2735MapHandler(const j2735_msgs::BSMConstPtr& message);

  /**
   * TODO
   * @brief Handles outbound messages from the ROS network
   * @param message
   *
   * This method packs the message according to the J2375 2016 standard,
   * and sends it to the client program
   */
  void systemAlertHandler(const cav_msgs::SystemAlertConstPtr& message);
};