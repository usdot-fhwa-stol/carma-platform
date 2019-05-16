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
import gov.dot.fhwa.saxton.carma.geometry.cartesian.Point3D;
import gov.dot.fhwa.saxton.carma.rosutils.MobilityHelper;
import gov.dot.fhwa.saxton.carma.rosutils.SaxtonLogger;

/**
 * Entry state for rsu metering operation
 * Waits for a vehicle to request to merge. If that vehicle is in a valid location control begins.
 */
public class StandbyState extends RSUMeteringStateBase {
  protected final static String EXPECTED_REQUEST_PARAMS = "MERGE|MAX_ACCEL:%.2f,LAG:%.2f,DIST:%.2f";
  protected final static String BROADCAST_MERGE_PARAMS = "INFO|RADIUS:%.2f,MERGE_DIST:%.2f,MERGE_LENGTH:%.2f";
  protected final static String MERGE_REQUEST_TYPE = "MERGE";
  protected final static List<String> MERGE_REQUEST_PARAMS = new ArrayList<>(Arrays.asList("MAX_ACCEL", "LAG", "DIST"));
  protected final static double CM_PER_M = 100.0;

  public StandbyState(RSUMeterWorker worker, SaxtonLogger log) {
    super(worker, log, worker.getRequestPeriod(), Long.MAX_VALUE);
  }

  @Override
  public boolean onMobilityRequestMessage(MobilityRequest msg) {

    String senderId = msg.getHeader().getSenderId();
    String planId = msg.getHeader().getPlanId();
    List<String> requestParams;
    try {
      requestParams = MobilityHelper.extractStrategyParams(msg.getStrategyParams(), MERGE_REQUEST_TYPE, MERGE_REQUEST_PARAMS);
    } catch (IllegalArgumentException e) {
      log.warn("Bad request strategy string received. Generated exception: " + e);
      return false;
    }

    double maxAccel = Double.parseDouble(requestParams.get(0));
    double lagTime = Double.parseDouble(requestParams.get(1));
    double vehicleDistToMerge = Double.parseDouble(requestParams.get(2));


    double distToMeter = vehicleDistToMerge - worker.getDistToMerge();

    // Check if vehicle is close enough to control
    if (distToMeter > worker.getMeterRadius()) {
      log.info("Received request to merge from vehicle id " + senderId +
       " but distance of " + distToMeter + " was greater than radius of " + worker.getMeterRadius());

      return false;
    }

    worker.setState(this, new HoldingState(worker, log, senderId, planId, lagTime, maxAccel, vehicleDistToMerge));
    return true;
  }

  @Override
  public void onMobilityOperationMessage(MobilityOperation msg) {
    // Do nothing the data updates will be handled by the worker class
  }

  @Override
  public void onMobilityResponseMessage(MobilityResponse msg) {
    // Do nothing
  }

  @Override
  protected void onLoop() {
    publishMergeLocationRequests();
  }

  @Override
  protected void onTimeout() {
    // Cannot timeout in this state
  }

  protected void publishMergeLocationRequests() {
    MobilityRequest msg = messageFactory.newFromType(MobilityRequest._TYPE);

    msg.getHeader().setRecipientId(worker.BROADCAST_ID);
    msg.getHeader().setSenderId(worker.getRsuId());
    msg.setStrategy(RSUMeterWorker.COOPERATIVE_MERGE_STRATEGY);
    msg.setStrategyParams(
      String.format(BROADCAST_MERGE_PARAMS, worker.getMeterRadius(), worker.getDistToMerge(), worker.getMergeLength())
    );

    Point3D meterECEF = worker.getMeterECEF();
    msg.getLocation().setEcefX((int)(meterECEF.getX() * CM_PER_M));
    msg.getLocation().setEcefY((int)(meterECEF.getY() * CM_PER_M));
    msg.getLocation().setEcefZ((int)(meterECEF.getZ() * CM_PER_M));

    worker.getManager().publishMobilityRequest(msg);
  }
}