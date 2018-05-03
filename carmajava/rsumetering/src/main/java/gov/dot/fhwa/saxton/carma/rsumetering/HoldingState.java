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
public class HoldingState extends RSUMeteringStateBase {
  protected final static String EXPECTED_OPERATION_PARAMS = "STATUS|METER_DIST:%.2f,MERGE_DIST:%.2f,SPEED:%.2f,LANE:%d";
  protected final static String STATUS_TYPE_PARAM = "STATUS";
  protected final static List<String> OPERATION_PARAMS = new ArrayList<>(Arrays.asList("METER_DIST", "MERGE_DIST", "SPEED", "LANE"));
  protected final double vehLagTime;
  protected final double vehMaxAccel;
  protected final String vehicleId;
  protected final String planId;
  protected double distToMerge;

  public HoldingState(RSUMeterWorker worker, SaxtonLogger log, String vehicleId, String planId, double vehLagTime, double vehMaxAccel, double distToMerge) {
    super(worker, log);
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
      
      updateCommands(0, vehMaxAccel, 0);

      if (speed < 0.1) {
        // Wait for a platoon to be incoming. Then transition to controlling state
        PlatoonData nextPlatoon = worker.getNextPlatoon();
        if (nextPlatoon != null) {
          long vehTimeTillMerge = worker.expectedTravelTime(mergeDist, speed, nextPlatoon.getSpeed(), vehLagTime, vehMaxAccel);
          long vehicleArrivalTime = System.currentTimeMillis() + vehTimeTillMerge;

          if (vehicleArrivalTime > nextPlatoon.getExpectedTimeOfArrival()
              && vehicleArrivalTime < nextPlatoon.getExpectedTimeOfArrival() + worker.getTimeMargin()) {

            log.info("Releasing vehicle with expected arrival time of " + vehicleArrivalTime +
             " and platoon arrival time of " + nextPlatoon.getExpectedTimeOfArrival());

             worker.setState(new CommandingState(worker, log, vehicleId, planId, vehLagTime, vehMaxAccel, distToMerge));

          } else if (vehicleArrivalTime > nextPlatoon.getExpectedTimeOfArrival() + worker.getTimeMargin()) {

            log.warn("Vehicle cannot reach merge before platoon passes but will try anyway");
            worker.setState(new CommandingState(worker, log, vehicleId, planId, vehLagTime, vehMaxAccel, distToMerge));

          } else {
            log.debug("Holding vehicle for platoon");
          }

        }
      }
      return; 
    }
    
    double neededAccel = -(speed * speed) / (2 * meterDist);

    if (neededAccel < -vehMaxAccel) {
      // We can't stop before merge point so command stop and reevaluate when stopped
      updateCommands(0, vehMaxAccel, 0);
      return;
    }

    // Request 0 speed but use max accel to limit behavior
    updateCommands(0, neededAccel, 0);
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
      worker.setState(new StandbyState(worker, log));
    }
  }

  @Override
  protected void onLoop() {
    this.publishSpeedCommand(vehicleId, planId);
  }
}