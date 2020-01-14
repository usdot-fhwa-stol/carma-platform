#pragma once
/*
 * Copyright (C) 2018-2020 LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
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
