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
import gov.dot.fhwa.saxton.carma.geometry.cartesian.Point3D;
import gov.dot.fhwa.saxton.carma.rosutils.SaxtonLogger;
import gov.dot.fhwa.saxton.carma.rsumetering.IRSUMeteringState;

/**
 * Struct for storing data about a RSU Ramp Metering infrastructure component
 */
public class StandbyState extends RSUMeteringStateBase {
  protected final static String EXPECTED_REQUEST_PARAMS = "MERGE|MAX_ACCEL:%.2f,LAG:%.2f,DIST:%.2f";
  protected final static String MERGE_REQUEST_TYPE = "MERGE";
  protected final static List<String> MERGE_REQUEST_PARAMS = new ArrayList<>(Arrays.asList("MAX_ACCEL", "LAG", "DIST"));

  public StandbyState(RSUMeterWorker worker, SaxtonLogger log) {
    super(worker, log);
  }

  @Override
  public boolean onMobilityRequestMessage(MobilityRequest msg) {
    // TODO We can assume that any message passed to a state is intended for us
    String senderId = msg.getHeader().getSenderId();
    String planId = msg.getHeader().getPlanId();
    List<String> requestParams;
    try {
      requestParams = worker.extractStrategyParams(msg.getStrategyParams(), MERGE_REQUEST_TYPE, MERGE_REQUEST_PARAMS);
    } catch (IllegalArgumentException e) {
      log.warn("Bad request strategy string received. Generated exception: " + e);
      return false;
    }

    double maxAccel = Double.parseDouble(requestParams.get(0));
    double lagTime = Double.parseDouble(requestParams.get(1));
    double vehicleDistToMerge = Double.parseDouble(requestParams.get(2));


    double distToMeter = vehicleDistToMerge - worker.getDistToMerg();

    // Check if vehicle is close enough to control
    if (distToMeter > worker.getMeterRadius()) {
      log.info("Received request to merge from vehicle id " + senderId +
       " but distance of " + distToMeter + " was greater than radius of " + worker.getMeterRadius());

      return false;
    }

    worker.setState(new HoldingState(worker, log, senderId, planId, lagTime, maxAccel, vehicleDistToMerge));
    return true;
  }

  @Override
  public void onMobilityOperationMessage(MobilityOperation msg) {
    // Do nothing the data updates will be handled by the worker class
  }

  @Override
  public void onMobilityResponseMessage(MobilityResponse msg) {
    // TODO We can assume that any message passed to a state is intended for us

    
  }

  @Override
  protected void onLoop() {
    // TODO publish mobility request broadcasts
  }
}