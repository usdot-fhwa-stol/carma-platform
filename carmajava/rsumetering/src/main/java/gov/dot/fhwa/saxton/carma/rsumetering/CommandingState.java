/*
 * Copyright (C) 2018-2019 LEIDOS.
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

import cav_msgs.MobilityOperation;
import cav_msgs.MobilityRequest;
import cav_msgs.MobilityResponse;
import gov.dot.fhwa.saxton.carma.rosutils.MobilityHelper;
import gov.dot.fhwa.saxton.carma.rosutils.SaxtonLogger;

/**
 * In this state the merging vehicle is being directly commanded by the rsu
 * At the moment the logic is very simple.
 * Set the target vehicle speed to the platoon speed and activate the lane change indicator when needed
 */
public class CommandingState extends RSUMeteringStateBase {
  protected static final double MIN_METER_DIST = -10.0; //meters; allow some buffer around meter location
  protected static final double MAX_METER_DIST = 1000.0; //meters
  protected static final double MIN_MERGE_DIST = -800.0; //meters
  protected static final double MAX_MERGE_DIST = 800.0; //meters
  protected static final double MIN_SPEED = 0.5; // m/s
  protected static final double MAX_SPEED = 35.0; // m/s - just over 75 mph
  protected static final int    MIN_LANE = 0;
  protected static final int    MAX_LANE = 5;

  protected final static String EXPECTED_OPERATION_PARAMS = "STATUS|METER_DIST:%.2f,MERGE_DIST:%.2f,SPEED:%.2f,LANE:%d";
  protected final static String STATUS_TYPE_PARAM = "STATUS";
  protected final static List<String> OPERATION_PARAMS = new ArrayList<>(Arrays.asList("METER_DIST", "MERGE_DIST", "SPEED", "LANE"));
  protected final double vehMaxAccel;
  protected final String vehicleId;
  protected final String planId;
  protected double distToMerge;

  /**
   * Constructor
   * 
   * @param worker The worker being represented by this state
   * @param log A logger
   * @param vehicleId The static id of the vehicle being controlled
   * @param vehMaxAccel The maximum acceleration limit allowed by the controlled vehicle
   * @param distToMerge The distance to the merge point of the controlled vehicle. This value can be negative
   * @param initialTargetSpeed The initial speed we are trying to achieve
   */
  public CommandingState(RSUMeterWorker worker, SaxtonLogger log, String vehicleId, String planId, 
    double vehMaxAccel, double distToMerge, double initialTargetSpeed) {
    super(worker, log, worker.getCommandPeriod(), worker.getCommsTimeout());
    this.vehMaxAccel = vehMaxAccel;
    this.distToMerge = distToMerge;
    this.vehicleId = vehicleId;
    this.planId = planId;

    updateCommands(initialTargetSpeed, vehMaxAccel, 0); // Ensure next command is consistent with entry configuration
    this.resetTimeout();
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
      params = MobilityHelper.extractStrategyParams(msg.getStrategyParams(), STATUS_TYPE_PARAM, OPERATION_PARAMS);
    } catch (IllegalArgumentException e) {
      log.warn("Received operation message with bad params. Exception: " + e);
      return;
    }

    // Reset our comms timeout
    resetTimeout();

    // Extract data
    double meterDist = Double.parseDouble(params.get(0));
    double mergeDist = Double.parseDouble(params.get(1));
    double speed = Double.parseDouble(params.get(2));
    int lane = Integer.parseInt(params.get(3));
    // Simply updating the command speed to the platoon speed may be enough to make this work
    // If it is not more complex logic can be added

    //perform sanity check on incoming params
    if (meterDist < MIN_METER_DIST  ||  meterDist > MAX_METER_DIST  ||
        mergeDist < MIN_MERGE_DIST  ||  mergeDist > MAX_MERGE_DIST  ||
        speed < MIN_SPEED           ||  speed > MAX_SPEED  ||
        lane < MIN_LANE             ||  lane > MAX_LANE) {
      log.warn("Received operation message with suspect strategy variables. meterDist = " + meterDist +
                ", mergeDist = " + mergeDist + ", speed = " + speed + ", lane = " + lane);
      return;
    }

    PlatoonData platoon = worker.getNextPlatoon(vehicleId);
    double newCommandSpeed;
    // Check if we have a platoon still
    if (platoon == null) {
      log.warn("No future platoon seen. Platoon may have already passed merge point. Continuing to merge without conflict");
      newCommandSpeed = this.getSpeedCommand();
    } else {
      newCommandSpeed = platoon.getSpeed();
    }
    // Target steering command
    double targetSteer = 0;

    // If we are not in our target lane and we are in the merge area, apply steering command
    if (lane != worker.getTargetLane() && mergeDist < 0 && Math.abs(mergeDist) < worker.getMergeLength()) {
      // With fake lateral control a positive value results in right lane change and negative in left lane change
      targetSteer = lane - worker.getTargetLane();
    }
    // Update vehicle commands
    updateCommands(newCommandSpeed, vehMaxAccel, targetSteer);
  }

  @Override
  public void onMobilityResponseMessage(MobilityResponse msg) {

    if (!msg.getHeader().getSenderId().equals(vehicleId)
      || !msg.getHeader().getPlanId().equals(planId)) {
        return;
    }

    if (!msg.getIsAccepted()) {
      log.warn("NACK received from vehicle: " + vehicleId + " for plan: " + planId);
      worker.setState(this, new StandbyState(worker, log));
    }
  }

  @Override
  protected void onLoop() {
    publishSpeedCommand(vehicleId, planId);
  }

  @Override
  protected void onTimeout() {
    log.warn("Timeout detected");
    // Send nack
    MobilityResponse msg = messageFactory.newFromType(MobilityResponse._TYPE);
    msg.getHeader().setPlanId(planId);
    msg.getHeader().setRecipientId(vehicleId);
    msg.getHeader().setSenderId(worker.getRsuId());
    msg.getHeader().setTimestamp(System.currentTimeMillis());

    msg.setIsAccepted(false);
    
    worker.getManager().publishMobilityResponse(msg);
    // Transition to standby state
    worker.setState(this, new StandbyState(worker, log));
  }
}