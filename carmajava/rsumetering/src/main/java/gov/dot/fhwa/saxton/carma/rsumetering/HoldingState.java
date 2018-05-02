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

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;

import com.google.common.util.concurrent.AtomicDouble;

import org.ros.message.MessageFactory;
import org.ros.node.NodeConfiguration;

import cav_msgs.MobilityOperation;
import cav_msgs.MobilityRequest;
import cav_msgs.MobilityResponse;
import gov.dot.fhwa.saxton.carma.geometry.cartesian.Point3D;
import gov.dot.fhwa.saxton.carma.rosutils.SaxtonLogger;
import gov.dot.fhwa.saxton.carma.rsumetering.IRSUMeteringState;

/**
 * Struct for storing data about a RSU Ramp Metering infrastructure component
 */
public class HoldingState implements IRSUMeteringState {
  protected final RSUMeterWorker worker;
  protected final static String EXPECTED_OPERATION_PARAMS = "STATUS|METER_DIST:%.2f,MERGE_DIST:%.2f,SPEED:%.2f";
  protected final static String STATUS_TYPE_PARAM = "STATUS";
  protected final static List<String> OPERATION_PARAMS = new ArrayList<>(Arrays.asList("METER_DIST", "MERGE_DIST", "SPEED"));
  protected final static String COMMAND_PARAMS = "COMMAND|SPEED:%.2f,STEERING_ANGLE:%.2f";
  protected final SaxtonLogger log;
  protected final double vehLagTime;
  protected final double vehMaxAccel;
  protected final String vehicleId;
  protected final String planId;
  protected double distToMerge;
  protected final double commandPeriod = 0.1; // Time between commands being sent

  protected AtomicDouble commandSpeed = new AtomicDouble(0); // TODO probably not safe to send 0 as default
  protected AtomicDouble commandSteer = new AtomicDouble(0); 

  protected AtomicBoolean readyToMerge = new AtomicBoolean(false);

  protected final MessageFactory messageFactory = NodeConfiguration.newPrivate().getTopicMessageFactory();

  public HoldingState(RSUMeterWorker worker, SaxtonLogger log, String vehicleId, String planId, double vehLagTime, double vehMaxAccel, double distToMerge) {
    this.worker = worker;
    this.log = log;
    this.vehLagTime = vehLagTime;
    this.vehMaxAccel = vehMaxAccel;
    this.distToMerge = distToMerge;
    this.vehicleId = vehicleId;
    this.planId = planId;
  }

  @Override
  public boolean onMobilityRequestMessage(MobilityRequest msg) {
    // Do nothing. We should not be getting requests in this state
    return false;
  }

  @Override
  public void onMobilityOperationMessage(MobilityOperation msg) {

    // Check this message is for the current merge plan
    if (!msg.getHeader().getSenderId().equals(vehicleId)
     || !msg.getHeader().getPlanId().equals(planId)) {
      return;
    }
    // Extract params
    List<String> params;
    try {
      params = worker.extractStrategyParams(msg.getStrategyParams(), STATUS_TYPE_PARAM, OPERATION_PARAMS);
    } catch (IllegalArgumentException e) {
      log.warn("Received operation message with bad params. Exception: " + e);
      return;
    }

    double meterDist = Double.parseDouble(params.get(0));
    double mergeDist = Double.parseDouble(params.get(1));
    double speed = Double.parseDouble(params.get(2));

    // If we are already at the ramp meter or past it then hold there
    if (meterDist < 0.5) {
      commandSpeed.set(0);
      if (speed < 0.1) {
        // Wait for a platoon to be incoming. Then transition to controlling state
        PlatoonData nextPlatoon = worker.getNextPlatoon();
        if (nextPlatoon != null) {
          
          worker.setState(new CommandingState(worker, log, vehicleId, planId, vehLagTime, vehMaxAccel, distToMerge));
        }
      }
      return; 
    }
    
    double neededAccel = -(speed * speed) / (2 * meterDist);

    if (neededAccel < -vehMaxAccel) {
      // We can't stop before merge point so command stop and reevaluate when stopped
      commandSpeed.set(0);
      return;
    }

    double targetSpeed = speed + neededAccel * commandPeriod;
    
    commandSpeed.set(targetSpeed);
  }

  @Override
  public void onMobilityResponseMessage(MobilityResponse msg) {
    // TODO We can assume that any message passed to a state is intended for us

    if (!msg.getHeader().getSenderId().equals(vehicleId)
      || !msg.getHeader().getPlanId().equals(planId)) {
        return;
    }

    if (!msg.getIsAccepted()) {
      log.warn("NACK received from vehicle: " + vehicleId + " for plan: " + planId);
      worker.setState(new StandbyState(worker, log));
      // TODO might be worth echoing the nack
    }
  }

  @Override
  public void loop() throws InterruptedException {
    long startTime = System.currentTimeMillis();
    publishSpeedCommand();
    long endTime = System.currentTimeMillis();
    Thread.sleep((long)(commandPeriod * 1000) - (endTime - startTime));
  }

  protected void publishSpeedCommand() {
    MobilityOperation msg = messageFactory.newFromType(MobilityOperation._TYPE);

    msg.getHeader().setPlanId(planId);
    msg.getHeader().setSenderId(worker.getRsuId());
    msg.getHeader().setRecipientId(vehicleId);
    msg.getHeader().setTimestamp(System.currentTimeMillis());
    
    msg.setStrategy(RSUMeterWorker.COOPERATIVE_MERGE_STRATEGY);
    msg.setStrategyParams(String.format(COMMAND_PARAMS, commandSpeed.get(), commandSteer.get()));
  
    worker.publishMobilityOperation(msg);
  }
}