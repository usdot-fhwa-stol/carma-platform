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

#include <cstdint>
#include <j2735_msgs/BSM.h>
#include <j2735_msgs/SPAT.h>
#include <j2735_msgs/MapData.h>
#include <cav_msgs/SystemAlert.h>
#include <cav_msgs/BSM.h>
#include <cav_msgs/SPAT.h>
#include <cav_msgs/MapData.h>
#include "units.h"



/**
 * @class BSMConvertor
 * @brief Is the class responsible for converting J2735 BSMs to CARMA usable BSMs
 */
class ValueConvertor 
{
    // TODO discuss need for extra parameters
  public:
    template<typename T, typename U, typename V, typename W, typename X>
    static T valueJ2735ToCav(const U in, const double conversion_factor,
      V& presence_vector, const W presence_flag, const X unavailability_value) {
      
      if (in != (U)unavailability_value) { // If the value is available
        presence_vector |= (V)presence_flag; // Mark the field as available
        return (T)(in / conversion_factor); // Do unit conversion
      } else {
        presence_vector &= ~presence_flag; // Mark the field as unavailable
        return 0; // Return ROS default of 0 for all fields
      }
    }

    template<typename T, typename U, typename V, typename W, typename X>
    static T valueCavToJ2735(const U in, const double conversion_factor,
      const V presence_vector, const W presence_flag, const X unavailability_value) {
      
      if (presence_vector & (V)presence_flag) { // Check if the field is available
        return (T)(in * conversion_factor); // Do the conversion
      } else {
        return (T)unavailability_value; // If field is unavailble return the unavailable flag
      }
    }

};
