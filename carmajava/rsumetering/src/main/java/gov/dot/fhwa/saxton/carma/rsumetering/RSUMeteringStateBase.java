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


import java.util.concurrent.atomic.AtomicLong;

import org.ros.message.MessageFactory;
import org.ros.node.NodeConfiguration;

import cav_msgs.MobilityOperation;
import cav_msgs.MobilityRequest;
import cav_msgs.MobilityResponse;
import gov.dot.fhwa.saxton.carma.rosutils.SaxtonLogger;
import gov.dot.fhwa.saxton.carma.rsumetering.IRSUMeteringState;

/**
 * Abstract base class for RSU Metering states
 * 
 * Provides final methods for publishing commands to a controlled vehicle in a synchronous fashion
 */
public abstract class RSUMeteringStateBase implements IRSUMeteringState {
  protected final RSUMeterWorker worker;
  protected final SaxtonLogger log;
  protected final long loopPeriod; // Time in ms between loop spins
  protected final long commsTimeout;
  private final Object commandMutex = new Object();
  private volatile double speedCommand = 5;
  private volatile double steerCommand = 0; 
  private volatile double maxAccelCommand = 2.5; 
  private AtomicLong lastMessageTime = new AtomicLong(0);
  private long lastCompletionTime = System.currentTimeMillis();
  protected final static String COMMAND_PARAMS = "COMMAND|SPEED:%.2f,ACCEL:%.2f,STEERING_ANGLE:%.2f";

  protected final MessageFactory messageFactory = NodeConfiguration.newPrivate().getTopicMessageFactory();

  public RSUMeteringStateBase(RSUMeterWorker worker, SaxtonLogger log, long loopPeriod, long commsTimeout) {
    this.worker = worker;
    this.log = log;
    this.loopPeriod = loopPeriod;
    this.commsTimeout = commsTimeout;
  }

  @Override
  public abstract boolean onMobilityRequestMessage(MobilityRequest msg);

  @Override
  public abstract void onMobilityOperationMessage(MobilityOperation msg);

  @Override
  public abstract void onMobilityResponseMessage(MobilityResponse msg);

  @Override
  public final void loop() throws InterruptedException {
    onLoop();
    checkTimeout();
    long doneTime = System.currentTimeMillis();
    Thread.sleep(Math.max(loopPeriod - (doneTime - lastCompletionTime), 0));
    lastCompletionTime = System.currentTimeMillis(); // Used to take into consideration the caller's spin rate when timing
  }

  /**
   * Calls the onTimeout function if a timeout has occurred
   */
  private void checkTimeout() {
    if (System.currentTimeMillis() - lastMessageTime.get() > commsTimeout) {
      this.onTimeout();
    }
  }

  /**
   * Function which will be called at a frequency equal to 1 / loopPeriod
   */
  protected abstract void onLoop();

  /**
   * Function which will be called when a timeout has occurred
   */
  protected abstract void onTimeout();

  /**
   * Resets the timeout count
   */
  protected final void resetTimeout() {
    lastMessageTime.set(System.currentTimeMillis());
  }

  /**
   * Function updates the command parameters in a thread sage manner
   * 
   * @param speed The speed command in m/s
   * @param maxAccel The max acceleration command in m/s^2
   * @param steer The steering angle in rad
   */
  protected void updateCommands(double speed, double maxAccel, double steer) {
    maxAccel = Math.abs(maxAccel); // Max accel should always be positive in CARMA
    synchronized(commandMutex) { // Synchronized to ensure commands are read as a set
      speedCommand = speed;
      maxAccelCommand = maxAccel;
      steerCommand = steer;
    }
  }

  /**
   * Function publishes the current command inputs (speed, max accel, and steer)
   * 
   * @param vehicleId The vehicle to send the command to
   * @param planId The plan id of the current merge operation
   */
  protected void publishSpeedCommand(String vehicleId, String planId) {
    MobilityOperation msg = messageFactory.newFromType(MobilityOperation._TYPE);

    msg.getHeader().setPlanId(planId);
    msg.getHeader().setSenderId(worker.getRsuId());
    msg.getHeader().setRecipientId(vehicleId);
    msg.getHeader().setTimestamp(System.currentTimeMillis());
    
    msg.setStrategy(RSUMeterWorker.COOPERATIVE_MERGE_STRATEGY);

    double speed, accel, steer; // Cache current commands

    synchronized (commandMutex) {
      speed = speedCommand;
      accel = maxAccelCommand;
      steer = steerCommand;
    }
    
    msg.setStrategyParams(String.format(COMMAND_PARAMS, speed, accel, steer));
  
    worker.getManager().publishMobilityOperation(msg);
  }

  /**
   * @return the maxAccelCommand
   */
  public double getMaxAccelCommand() {
    return maxAccelCommand;
  }

  /**
   * @return the speedCommand
   */
  public double getSpeedCommand() {
    return speedCommand;
  }
  
  /**
   * @return the steerCommand
   */
  public double getSteerCommand() {
    return steerCommand;
  }
}