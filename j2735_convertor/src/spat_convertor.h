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
#include "units.h"

/**
 * @class SPATConvertor
 * @brief Is the class responsible for converting J2735 BSMs to CARMA usable BSMs
 */
class SPATConvertor 
{
  public:
    /**
     * @brief constructor
     * @param argc - command line argument count
     * @param argv - command line arguments
     */
    SPATConvertor() {};

    ~SPATConvertor() 
    {
      //TODO
    }

    static void convert(const j2735_msgs::SPAT& in_msg, cav_msgs::SPAT& out_msg);

  private:
    static void convertTimeChangeDetails(const j2735_msgs::TimeChangeDetails& in_msg, cav_msgs::TimeChangeDetails& out_msg);

    static void convertAdvisorySpeed(const j2735_msgs::AdvisorySpeed& in_msg, cav_msgs::AdvisorySpeed& out_msg);

    static void convertMovementEvent(const j2735_msgs::MovementEvent& in_msg, cav_msgs::MovementEvent& out_msg);

    static void convertMovementState(const j2735_msgs::MovementState& in_msg, cav_msgs::MovementState& out_msg);

    static void convertIntersectionState(const j2735_msgs::IntersectionState& in_msg, cav_msgs::IntersectionState& out_msg);
};  