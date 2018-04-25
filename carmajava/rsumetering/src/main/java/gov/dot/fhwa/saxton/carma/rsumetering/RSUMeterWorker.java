/*
 * Copyright (C) 2018 LEIDOS.
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

package gov.dot.fhwa.saxton.carma.rsumetering;

import gov.dot.fhwa.saxton.carma.rosutils.SaxtonLogger;
import org.ros.message.MessageFactory;
import org.ros.node.NodeConfiguration;
import cav_msgs.MobilityOperation;
 
/**
 * Primary logic class for the RSUMeterManager node
 */
public class RSUMeterWorker {

  protected IRSUMeterManager manager;
  protected SaxtonLogger log;
  protected final NodeConfiguration nodeConfiguration = NodeConfiguration.newPrivate();
  protected final MessageFactory messageFactory = nodeConfiguration.getTopicMessageFactory();

  /**
   * Constructor
   * 
   * @param manager IRSUMeterManager responsible for providing timing and publishing capabilities
   * @param log Logging object
   */
  RSUMeterWorker(IRSUMeterManager manager, SaxtonLogger log) {
    this.manager = manager;
    this.log = log;
  }

  /**
   * Handles control messages. 
   * If the requested axleAngle is greater than the angleThreshold then it will be considered a request for a turn.
   * The UI will then be notified. 
   */
  public void handleMobilityOperationMsg(MobilityOperation msg) {

  }
}


