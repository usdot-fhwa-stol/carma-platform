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
 
public class LateralControlWorker {

    protected ILateralControlDriver driver_;
    protected SaxtonLogger log_;
    protected int angleThreshold_;
    protected final NodeConfiguration nodeConfiguration = NodeConfiguration.newPrivate();
    protected final MessageFactory messageFactory = nodeConfiguration.getTopicMessageFactory();

    LateralControlWorker(ILateralControlDriver driver, SaxtonLogger log, double angleThreshold) {
      driver_ = driver;
      log_ = log;
      angleThreshold_ = angleThreshold;
    }

    public void handleLateralControlMsg(cav_msgs.LateralControl msg) {
        if (msg.getAxelAngle() < -angleThreshold_) { // left turn
          cav_msgs.UIInstructions uiMsg = messageFactory.newFromType(cav_msgs.UIInstructions._TYPE);
          uiMsg.setMsg("LEFT_TURN");
          uiMsg.setType(cav_msgs.UIInstructions.NO_ACK_REQUIRED);
          uiMsg.setStamp(driver_.getTime());
          driver_.publishUIMessage(uiMsg);
          log_.info("Left turn requested with axis angle of " + msg.getAxelAngle());

        } else if (msg.getAxelAngle() > angleThreshold_) { // right turn
          cav_msgs.UIInstructions uiMsg = messageFactory.newFromType(cav_msgs.UIInstructions._TYPE);
          uiMsg.setMsg("RIGHT_TURN");
          driver_.publishUIMessage(uiMsg);
          uiMsg.setType(cav_msgs.UIInstructions.NO_ACK_REQUIRED);
          uiMsg.setStamp(driver_.getTime());
          log_.info("Right turn requested with axis angle of " + msg.getAxelAngle());
        }
    }
}


