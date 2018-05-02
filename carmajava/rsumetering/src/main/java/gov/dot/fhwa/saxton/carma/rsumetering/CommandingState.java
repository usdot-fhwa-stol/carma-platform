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

import cav_msgs.MobilityOperation;
import cav_msgs.MobilityRequest;
import cav_msgs.MobilityResponse;
import gov.dot.fhwa.saxton.carma.rosutils.SaxtonLogger;

/**
 * Struct for storing data about a RSU Ramp Metering infrastructure component
 */
public class CommandingState extends RSUMeteringStateBase {
  protected final static String EXPECTED_OPERATION_PARAMS = "STATUS|METER_DIST:%.2f,MERGE_DIST:%.2f,SPEED:%.2f";
  protected final static String STATUS_TYPE_PARAM = "STATUS";
  protected final static List<String> OPERATION_PARAMS = new ArrayList<>(Arrays.asList("METER_DIST", "MERGE_DIST", "SPEED"));
  protected final double vehLagTime;
  protected final double vehMaxAccel;
  protected final String vehicleId;
  protected final String planId;
  protected double distToMerge;

  public CommandingState(RSUMeterWorker worker, SaxtonLogger log, String vehicleId, String planId, double vehLagTime, double vehMaxAccel, double distToMerge) {
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

    // Simply updating the command speed to the platoon speed may be enough to make this work
    // If it is not more complex logic can be added
    PlatoonData platoon = worker.getNextPlatoon();

    // TODO need to give the steering command when pasted merge point
    updateCommands(platoon.getSpeed(), vehMaxAccel, 0);
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
  protected void onLoop() {
    publishSpeedCommand(vehicleId, planId);
  }
}