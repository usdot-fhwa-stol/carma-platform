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
 * State which holds a vehicle at the ramp metering location until it is time to release it
 */
public class HoldingState extends RSUMeteringStateBase {
  protected static final double MIN_METER_DIST = -10.0; //meters; allow buffer around meter location
  protected static final double MAX_METER_DIST = 1000.0; //meters
  protected static final double MIN_MERGE_DIST = -800.0; //meters
  protected static final double MAX_MERGE_DIST = 800.0; //meters
  protected static final double MIN_SPEED = 0.5; // m/s
  protected static final double MAX_SPEED = 35.0; // m/s - just over 75 mph

  protected final static String EXPECTED_OPERATION_PARAMS = "STATUS|METER_DIST:%.2f,MERGE_DIST:%.2f,SPEED:%.2f,LANE:%d";
  protected final static String STATUS_TYPE_PARAM = "STATUS";
  protected final static List<String> OPERATION_PARAMS = new ArrayList<>(Arrays.asList("METER_DIST", "MERGE_DIST", "SPEED", "LANE"));
  protected final double vehLagTime;
  protected final double combinedLagTime;
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
   * @param vehLagTime The lag time of the controlled vehicle's response
   * @param vehMaxAccel The maximum acceleration limit allowed by the controlled vehicle. Always positive
   * @param distToMerge The distance to the merge point of the controlled vehicle. This value can be negative
   */
  public HoldingState(RSUMeterWorker worker, SaxtonLogger log, String vehicleId, String planId, double vehLagTime, double vehMaxAccel, double distToMerge) {
    super(worker, log, worker.getCommandPeriod(), worker.getCommsTimeout());
    this.vehLagTime = vehLagTime;
    this.combinedLagTime = worker.getDriverLagTime() + worker.getCommsLagTime() + vehLagTime;
    this.vehMaxAccel = vehMaxAccel;
    this.distToMerge = distToMerge;
    this.vehicleId = vehicleId;
    this.planId = planId;

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

    //perform sanity checks on the received values
    if (meterDist < MIN_METER_DIST  ||  meterDist > MAX_METER_DIST  ||
        mergeDist < MIN_MERGE_DIST  ||  mergeDist > MAX_MERGE_DIST  ||
        speed < MIN_SPEED           ||  speed > MAX_SPEED) {
      log.warn("Received operation message with suspect strategy values. meterDist = " + meterDist +
                ", mergeDist = " + mergeDist + ", speed = " + speed);
      return;
    }

    // If we are already at the ramp meter or past it then hold there
    if (meterDist < 1.0) {
      
      updateCommands(0, vehMaxAccel, 0);
      log.info("Vehicle at meter point");
      if (speed < 0.1) {
        log.info("Vehicle at meter point and stopped");
        // Wait for a platoon to be incoming. Then transition to controlling state
        PlatoonData nextPlatoon = worker.getNextPlatoon(vehicleId);
        if (nextPlatoon != null) {
          long vehTimeTillMerge = worker.expectedTravelTime(mergeDist, speed, nextPlatoon.getSpeed(), combinedLagTime, vehMaxAccel);
          long vehicleArrivalTime = System.currentTimeMillis() + vehTimeTillMerge;

          if (vehicleArrivalTime > nextPlatoon.getExpectedTimeOfArrival()
              && vehicleArrivalTime < nextPlatoon.getExpectedTimeOfArrival() + worker.getTimeMargin()) {

            log.info("Releasing vehicle with expected arrival time of " + vehicleArrivalTime +
             " for platoon " + nextPlatoon);

            worker.setState(this, new CommandingState(worker, log, vehicleId, planId, vehMaxAccel, distToMerge, nextPlatoon.getSpeed()));

          } else if (vehicleArrivalTime > nextPlatoon.getExpectedTimeOfArrival() + worker.getTimeMargin()) {

            log.warn("Vehicle cannot reach merge before platoon passes but will try anyway");
            worker.setState(this, new CommandingState(worker, log, vehicleId, planId, vehMaxAccel, distToMerge, nextPlatoon.getSpeed()));
          
          } else {
            log.debug("Holding vehicle for platoon");
          }

        }
      }
      return; 
    }

    double targetSpeed = worker.getTargetApproachSpeed();

    double neededAccel = ((targetSpeed * targetSpeed) - (speed * speed)) / (2 * meterDist);

    log.debug("Needed accel: " + neededAccel);

    if (neededAccel < -vehMaxAccel) {
      // We can't stop before merge point so command stop and reevaluate when stopped
      updateCommands(targetSpeed, vehMaxAccel, 0);
      return;
    }

    // Ensure out maximum acceleration never falls below a comfortable threshold
    if (neededAccel > -worker.getMinApproachAccel()) {
      neededAccel = worker.getMinApproachAccel();
    }

    // Request 0 speed but use max accel to limit behavior
    updateCommands(targetSpeed, neededAccel, 0);
  }

  @Override
  public void onMobilityResponseMessage(MobilityResponse msg) {

    // Check this message is for the current merge plan
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
    this.publishSpeedCommand(vehicleId, planId);
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