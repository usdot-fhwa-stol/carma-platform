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

package gov.dot.fhwa.saxton.carma.lateralcontroldriver;

import gov.dot.fhwa.saxton.carma.rosutils.SaxtonLogger;
import org.apache.commons.logging.Log;
import gov.dot.fhwa.saxton.carma.rosutils.SaxtonBaseNode;
import org.ros.message.MessageFactory;
import org.ros.node.NodeConfiguration;
 
/**
 * Primary logic class for the LateralControlDriver
 * Allows for dipendency injection and unit testing
 */
public class LateralControlWorker {

    protected ILateralControlDriver driver_;
    protected SaxtonLogger log_;
    protected double angleThreshold_;
    protected final NodeConfiguration nodeConfiguration = NodeConfiguration.newPrivate();
    protected final MessageFactory messageFactory = nodeConfiguration.getTopicMessageFactory();

    /**
     * Constructor
     * 
     * @param driver Object responsible for providing timing and publishing capabilities
     * @param log Logging object
     * @param angleThreshold The threshold over which a requested angle will be considered as left or right turn. In radians
     */
    LateralControlWorker(ILateralControlDriver driver, SaxtonLogger log, double angleThreshold) {
      driver_ = driver;
      log_ = log;
      angleThreshold_ = angleThreshold;
    }

    /**
     * Handles control messages. 
     * If the requested axleAngle is greater than the angleThreshold then it will be considered a request for a turn.
     * The UI will then be notified. 
     */
    public void handleLateralControlMsg(cav_msgs.LateralControl msg) {
        if (msg.getAxleAngle() < -angleThreshold_) { // left lane change
          cav_msgs.UIInstructions uiMsg = messageFactory.newFromType(cav_msgs.UIInstructions._TYPE);
          uiMsg.setMsg("LEFT_LANE_CHANGE");
          uiMsg.setType(cav_msgs.UIInstructions.NO_ACK_REQUIRED);
          uiMsg.setStamp(driver_.getTime());
          driver_.publishUIMessage(uiMsg);
          log_.info("Left turn requested with axis angle of " + msg.getAxleAngle());

        } else if (msg.getAxleAngle() > angleThreshold_) { // right lane change
          cav_msgs.UIInstructions uiMsg = messageFactory.newFromType(cav_msgs.UIInstructions._TYPE);
          uiMsg.setMsg("RIGHT_LANE_CHANGE");
          driver_.publishUIMessage(uiMsg);
          uiMsg.setType(cav_msgs.UIInstructions.NO_ACK_REQUIRED);
          uiMsg.setStamp(driver_.getTime());
          log_.info("Right turn requested with axis angle of " + msg.getAxleAngle());
        }
    }

    /**
     * Function returns the current state of the driver
     */
    public byte getDriverStatus() {
      return cav_msgs.DriverStatus.OPERATIONAL;
    }
}


