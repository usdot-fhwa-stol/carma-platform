/*
 * TODO: Copyright (C) 2017 LEIDOS.
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

package gov.dot.fhwa.saxton.carma.route;

/**
 * An enumeration defining various road types such as a freeway, ramp, arterial, secondary, or parking lot.
 */
public enum RoadType {
  /**
   * Freeway (highway)
   */
  FREEWAY,
  /**
   * Ramp onto or off of a freeway
   */
  RAMP,
  /**
   * Arterial road (high capacity urban/sub-urban road)
   */
  ARTERIAL,
  /**
   * Secondary road (standard low capacity road)
   */
  SECONDARY,
  /**
   * A parking lot or unmarked paved region
   */
  PARKING_LOT;

  /**
   * Gets a RoadType object from a ros message of the corresponding test
   * @param type
   * @return
   */
  public static RoadType fromMessage(cav_msgs.RoadType type){
    switch (type.getType()){
      case cav_msgs.RoadType.FREEWAY:
        return RoadType.FREEWAY;
      case cav_msgs.RoadType.RAMP:
        return RoadType.RAMP;
      case cav_msgs.RoadType.ARTERIAL:
        return RoadType.ARTERIAL;
      case cav_msgs.RoadType.SECONDARY:
        return RoadType.SECONDARY;
      case cav_msgs.RoadType.PARKING_LOT:
        return RoadType.PARKING_LOT;
      default:
        return RoadType.FREEWAY;
    }
  }
}
