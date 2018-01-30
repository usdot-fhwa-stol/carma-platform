/*
 * Copyright (C) 2017 LEIDOS.
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

package gov.dot.fhwa.saxton.carma.roadway;

import org.ros.message.MessageFactory;
import org.ros.node.NodeConfiguration;

/**
 * An enumeration which identifies the different types of connected vehicles including those with no communication
 */
public enum ConnectedVehicleType {

  /**
   * Vehicle has no known communication capabilities
   */
  NOT_CONNECTED(cav_msgs.ConnectedVehicleType.NOT_CONNECTED),

  /**
   * Vehicle is known to be connected as BSMs are being recieved.
   * No other assumptions are made about its capabilities.
   */
  CONNECTED(cav_msgs.ConnectedVehicleType.CONNECTED),

  /**
   * CAV: Vehicle is known to be connected and automated
   */
  CONNECTED_AND_AUTOMATED(cav_msgs.ConnectedVehicleType.CONNECTED_AND_AUTOMATED);

  private byte value;

  ConnectedVehicleType(byte value) {
    this.value = value;
  }

  private byte getValue() {
    return value;
  }

  /**
   * Converts an instance of this enum into a ros message
   * @return the instantiated ros message
   */
  public cav_msgs.ConnectedVehicleType toMessage(){
    MessageFactory factory = NodeConfiguration.newPrivate().getTopicMessageFactory();
    cav_msgs.ConnectedVehicleType msg = factory.newFromType(cav_msgs.ConnectedVehicleType._TYPE);
    msg.setType(this.getValue());
    return msg;
  }

  /**
   * Creates an instance of this enum from a ros message
   * @param msg The message to convert
   * @return The equivalent enum
   */
  public static ConnectedVehicleType fromMessge(cav_msgs.ConnectedVehicleType msg) {
    switch (msg.getType()) {
      case cav_msgs.ConnectedVehicleType.NOT_CONNECTED:
        return NOT_CONNECTED;
      case cav_msgs.ConnectedVehicleType.CONNECTED:
        return CONNECTED;
      case cav_msgs.ConnectedVehicleType.CONNECTED_AND_AUTOMATED:
        return CONNECTED_AND_AUTOMATED;
      default:
        return NOT_CONNECTED;
    }
  }

}
