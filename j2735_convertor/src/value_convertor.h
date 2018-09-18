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
 * @class ValueConvertor
 * @brief Class responsible for converting between discrete values in j2735_msgs and cav_msgs
 * 
 * Provides templated functions which will perform unit conversions and casting.
 * Additionally, conversion between presence vectors and unavailability values is supported.
 */
class ValueConvertor 
{
  public:

    /**
     * @brief Converts values from j2735 standard to format used in cav_msgs
     * 
     * @tparam T Return type which the provided input should be converted to
     * @tparam U Type of the input value
     * @tparam V Type of the presence_vector defining if the output value represents actual data
     * @tparam W Type of the presence_flag this type should be the same as V but is kept seperate to support Enums
     * @tparam X Type of the unavailability_value which represents if the input value represents actual data. 
     *           This type should be the same as U but is kept seperate to support Enums
     * 
     * @param in The input value to convert the type and units of
     * @param conversion_factor The value to divide the input by to convert it to the desired units
     * @param presence_vector A bit flag which is set to identify if the output value represents actual data
     * @param presence_flag A flag with a single bit set to mark the presence vector as needed
     * @param unavailability_value The value of the input which would mean that value was not representative of actual data
     * 
     * @return The converted input value in the new output unit and type
     */
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

    /**
     * @brief Converts values from cav_msgs standard to the j2735 standard
     * 
     * @tparam T Return type which the provided input should be converted to
     * @tparam U Type of the input value
     * @tparam V Type of the presence_vector defining if the input value represents actual data
     * @tparam W Type of the presence_flag this type should be the same as V but is kept seperate to support Enums
     * @tparam X Type of the unavailability_value which represents if the output value represents actual data. 
     *           This type should be the same as U but is kept seperate to support Enums
     * 
     * @param in The input value to convert the type and units of
     * @param conversion_factor The value to multiply the input by to convert it to the desired units
     * @param presence_vector A bit flag which is set to identify if the input value represents actual data
     * @param presence_flag A flag with a single bit set to mark the presence vector as needed
     * @param unavailability_value The value of the output which would mean that value was not representative of actual data
     * 
     * @return The converted input value in the new output unit and type
     */
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
