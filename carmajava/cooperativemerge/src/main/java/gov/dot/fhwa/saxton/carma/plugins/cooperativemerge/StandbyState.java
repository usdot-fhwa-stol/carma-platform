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

package gov.dot.fhwa.saxton.carma.plugins.cooperativemerge;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ConcurrentMap;

import org.jboss.netty.util.internal.ConcurrentHashMap;
import org.ros.rosjava_geometry.Vector3;

import cav_msgs.MobilityOperation;
import cav_msgs.MobilityRequest;
import cav_msgs.MobilityResponse;
import gov.dot.fhwa.saxton.carma.geometry.cartesian.Point3D;
import gov.dot.fhwa.saxton.carma.guidance.arbitrator.TrajectoryPlanningResponse;
import gov.dot.fhwa.saxton.carma.guidance.mobilityrouter.MobilityRequestResponse;
import gov.dot.fhwa.saxton.carma.guidance.plugins.PluginServiceLocator;
import gov.dot.fhwa.saxton.carma.guidance.trajectory.Trajectory;
import gov.dot.fhwa.saxton.carma.guidance.util.ILogger;
import gov.dot.fhwa.saxton.carma.guidance.util.RouteService;
import gov.dot.fhwa.saxton.carma.rosutils.MobilityHelper;
import gov.dot.fhwa.saxton.carma.route.Route;
import gov.dot.fhwa.saxton.carma.route.RouteSegment;

/**
 * Standby state of the CooperativeMergePlugin. 
 * The plugin will remain in this state until a ramp metering rsu is detected
 */
public class StandbyState implements ICooperativeMergeState {
  
  protected final static int LOOP_SLEEP_TIME = 1000; // ms

  protected final static double CM_PER_M = 100.0;

  protected final static double MIN_RADIUS = 100.0; //meters
  protected final static double MAX_RADIUS = 10000.0; //meters
  protected final static double MIN_MERGE_DIST_FROM_METER = -10.0; //meters; allow buffer around meter point
  protected final static double MAX_MERGE_DIST_FROM_METER = 1000.0; //meters
  protected final static double MIN_MERGE_LENGTH = 5.0; //meters
  protected final static double MAX_MERGE_LENGTH = 800.0; //meters

  protected final CooperativeMergePlugin plugin;
  protected final ILogger log;
  protected final PluginServiceLocator pluginServiceLocator;
  protected final ConcurrentMap<String, RampMeterData> rampMeters = new ConcurrentHashMap<>();
  protected final String INFO_PARAM_TYPE = "INFO";
  protected final List<String> REQUEST_PARAMS_KEYS = new ArrayList<>(Arrays.asList("RADIUS", "MERGE_DIST", "MERGE_LENGTH"));
 

  /**
   * Constructor
   * 
   * @param plugin The cooperative merge plugin
   * @param log The logger to use
   * @param pluginServiceLocator Used to access host vehicle data
   */
  public StandbyState(CooperativeMergePlugin plugin, ILogger log, PluginServiceLocator pluginServiceLocator) {
    this.plugin               = plugin;
    this.log                  = log;
    this.pluginServiceLocator = pluginServiceLocator;
  }
  
  @Override
  public TrajectoryPlanningResponse planTrajectory(Trajectory traj, double expectedEntrySpeed) {
    // No need to plan trajectory in StandbyState
    TrajectoryPlanningResponse tpr = new TrajectoryPlanningResponse();
    return tpr;
  }

  @Override
  public MobilityRequestResponse onMobilityRequestMessage(MobilityRequest msg) {
    // In standby state, the plugin waits to receive a message from a ramp metering rsu
    // Parse Strategy Params
    // Expecting "INFO|RADIUS:%.2f,MERGE_DIST:%.2f,MERGE_LENGTH:%.2f";
    List<String> params;
    try {
      params = MobilityHelper.extractStrategyParams(msg.getStrategyParams(), INFO_PARAM_TYPE, REQUEST_PARAMS_KEYS);
    } catch (IllegalArgumentException e) {
      log.error("Received mobility request with invalid params. Exception: " + e);
      return MobilityRequestResponse.NO_RESPONSE;
    }

    // Get RSU Id
    String rsuId = msg.getHeader().getSenderId();

    // If this is our first time seeing this, RSU cache its information
    if (!rampMeters.containsKey(rsuId)) {
      double meterRadius       = Double.parseDouble(params.get(0));
      double mergeDTDFromMeter = Double.parseDouble(params.get(1));
      double mergeLength       = Double.parseDouble(params.get(2));

      //do a sanity check on the values
      if (meterRadius < MIN_RADIUS                        ||  meterRadius > MAX_RADIUS  ||
          mergeDTDFromMeter < MIN_MERGE_DIST_FROM_METER   ||  mergeDTDFromMeter > MAX_MERGE_DIST_FROM_METER  ||
          mergeLength < MIN_MERGE_LENGTH                  ||  mergeLength > MAX_MERGE_LENGTH) {
        log.error("Received mobility request with suspect strategy values. meterRadius = " + meterRadius +
                  ", mergeDTDFromMeter = " + mergeDTDFromMeter + ", mergeLength = " + mergeLength);
        return MobilityRequestResponse.NO_RESPONSE;
      }

      cav_msgs.LocationECEF meterLoc = msg.getLocation();
      // TODO Probably would be good to add a function to route for doing this complicated process which happens alot
      Point3D      meterPoint = new Point3D(meterLoc.getEcefX() / CM_PER_M, meterLoc.getEcefY() / CM_PER_M, meterLoc.getEcefZ() / CM_PER_M);
      Route        route      = pluginServiceLocator.getRouteService().getCurrentRoute();
      RouteSegment meterSeg   = route.routeSegmentOfPoint(meterPoint, route.getSegments());                   // TODO optimize this
      // Get the point in location of the meter in segment frame
      Vector3 meterPointInSeg = meterSeg.getECEFToSegmentTransform().invert().apply(new Vector3(meterPoint.getX(), meterPoint.getY(), meterPoint.getZ()));
      double  segmentDTD      = route.lengthOfSegments(0, meterSeg.getUptrackWaypoint().getWaypointId() - 1);
      double  meterDTD        = segmentDTD + meterPointInSeg.getX();
      double  mergeDTD        = meterDTD + mergeDTDFromMeter;

      // Thread safe atomic put
      RampMeterData newMeterData = new RampMeterData(rsuId, meterDTD, mergeDTD, mergeLength, meterRadius);
      rampMeters.putIfAbsent(rsuId, newMeterData);
    }
    
    // Get current downtrack distance to see if we are near the ramp meter point
    double currentDTD = pluginServiceLocator.getRouteService().getCurrentDowntrackDistance();
    // Get the data for the rsu which sent this message
    RampMeterData rsuData = rampMeters.get(rsuId);

    if (rsuData == null) {

      log.warn("RSU Data was not properly cached");
      return MobilityRequestResponse.NO_RESPONSE;

    } else if (Math.abs(rsuData.getRampMeterDTD() - currentDTD) < rsuData.getRadius()) {
      // Change to planning state
      plugin.setState(this, new PlanningState(plugin, log, pluginServiceLocator, rsuData));
    }
    return MobilityRequestResponse.NO_RESPONSE;
  }
  
  @Override
  public void onMobilityResponseMessage(MobilityResponse msg) {
    // In standby state, it will not send out any requests so it will also ignore all responses
  }
  
  @Override
  public void onMobilityOperationMessage(MobilityOperation msg) {
    // In standby state, it will ignore operation message since it is not actively operating
  }
  
  @Override
  public void loop() throws InterruptedException {
    Thread.sleep(LOOP_SLEEP_TIME);
  }
  
  @Override
  public String toString() {
    return this.getClass().getSimpleName();
  }
}
