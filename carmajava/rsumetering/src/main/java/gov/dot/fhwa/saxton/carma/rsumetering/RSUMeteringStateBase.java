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


import org.ros.message.MessageFactory;
import org.ros.node.NodeConfiguration;

import cav_msgs.MobilityOperation;
import cav_msgs.MobilityRequest;
import cav_msgs.MobilityResponse;
import gov.dot.fhwa.saxton.carma.rosutils.SaxtonLogger;
import gov.dot.fhwa.saxton.carma.rsumetering.IRSUMeteringState;

/**
 * Struct for storing data about a RSU Ramp Metering infrastructure component
 */
public abstract class RSUMeteringStateBase implements IRSUMeteringState {
  protected final RSUMeterWorker worker;
  protected final SaxtonLogger log;
  protected long commandPeriod; // TODO Time in ms between commands being sent

  private Object commandMutex = new Object();
  private volatile double speedCommand = 0; // TODO probably not safe to send 0 as default
  private volatile double steerCommand = 0; 
  private volatile double maxAccelCommand = 0; 
  protected final static String COMMAND_PARAMS = "COMMAND|SPEED:%.2f,ACCEL:%.2f,STEERING_ANGLE:%.2f";

  protected final MessageFactory messageFactory = NodeConfiguration.newPrivate().getTopicMessageFactory();

  public RSUMeteringStateBase(RSUMeterWorker worker, SaxtonLogger log) {
    this.worker = worker;
    this.log = log;
  }

  @Override
  public abstract boolean onMobilityRequestMessage(MobilityRequest msg);

  @Override
  public abstract void onMobilityOperationMessage(MobilityOperation msg);

  @Override
  public abstract void onMobilityResponseMessage(MobilityResponse msg);

  @Override
  public final void loop() throws InterruptedException {
    long startTime = System.currentTimeMillis();
    onLoop();
    long endTime = System.currentTimeMillis();
    Thread.sleep(commandPeriod - (endTime - startTime));
  }

  protected abstract void onLoop();

  protected void updateCommands(double speed, double maxAccel, double steer) {
    synchronized(commandMutex) { // Synchronized to ensure commands are read as a set
      speedCommand = speed;
      maxAccelCommand = maxAccel;
      steerCommand = steer;
    }
  }

  protected void publishSpeedCommand(String vehicleId, String planId) {
    MobilityOperation msg = messageFactory.newFromType(MobilityOperation._TYPE);

    msg.getHeader().setPlanId(planId);
    msg.getHeader().setSenderId(worker.getRsuId());
    msg.getHeader().setRecipientId(vehicleId);
    msg.getHeader().setTimestamp(System.currentTimeMillis());
    
    msg.setStrategy(RSUMeterWorker.COOPERATIVE_MERGE_STRATEGY);
    
    synchronized (commandMutex) {
      msg.setStrategyParams(String.format(COMMAND_PARAMS, speedCommand, maxAccelCommand, steerCommand));
    }
  
    worker.getManager().publishMobilityOperation(msg);
  }
}