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

package gov.dot.fhwa.saxton.carma.plugins.cooperativemerge;

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
import gov.dot.fhwa.saxton.carma.route.Route;
import gov.dot.fhwa.saxton.carma.route.RouteSegment;

/**
 * TODO
 * The StandbyState is a state when the platooning algorithm is current disabled on the route.
 * It will transit to SingleVehiclePlatoonState when it knows the algorithm will be enabled in the next trajectory.
 * In this state, the plug-in will not insert any maneuvers into a trajectory and will ignore all negotiation messages.
 */
public class StandbyState implements ICooperativeMergeState {
  
  protected final static int LOOP_SLEEP_TIME = 1000; // ms
  
  protected final CooperativeMergePlugin plugin;
  protected final ILogger log;
  protected final PluginServiceLocator pluginServiceLocator;
  protected final ConcurrentMap<String, RampMeterData> rampMeters = new ConcurrentHashMap<>();
  
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
    // TODO enhancement cache meter locations to improve performance
    // In standby state, the plugin waits to receive a message from a ramp metering rsu
    // Parse Strategy Params
    // Expected String "INFO|RADIUS:%.2f,MERGE_DIST:%.2f,MERGE_LENGTH:%.2f";  // TODO add distance to merge point
    final  String RADIUS_PARAM       = "RADIUS";
    final  String MERGE_DIST_PARAM   = "MERGE_DIST";
    final  String MERGE_LENGTH_PARAM = "MERGE_LENGTH";
    final  String INFO_TYPE          = "INFO";
    String paramsString              = msg.getStrategyParams();
    String[] paramsParts             = paramsString.split("\\|");
    String typeString                = paramsParts[0];
    String dataString                = paramsParts[1];
    String[] dataParts               = dataString.split(",");
    String[] radiusParts             = dataParts[0].split(":");
    String[] mergeDistParts          = dataParts[1].split(":");
    String[] mergeLengthParts        = dataParts[2].split(":");

    // Validate Params
    if (!typeString.equals(INFO_TYPE) || !radiusParts[0].equals(RADIUS_PARAM)
        || !mergeDistParts[0].equals(MERGE_DIST_PARAM)
        || !mergeLengthParts[0].equals(MERGE_LENGTH_PARAM)) {
        
      log.info("Received mobility request with invalid params for state. Params: " + paramsString);
      return MobilityRequestResponse.NO_RESPONSE;
    }

    // Get RSU Id
    String rsuId = msg.getHeader().getSenderId();

    // If this is our first time seeing this, RSU cache its information
    if (!rampMeters.containsKey(rsuId)) {
      double meterRadius       = Double.parseDouble(radiusParts[1]);
      double mergeDTDFromMeter = Double.parseDouble(mergeDistParts[1]);
      double mergeLength       = Double.parseDouble(mergeLengthParts[1]);

      cav_msgs.LocationECEF meterLoc = msg.getLocation();
      // TODO Probably would be good to add a function to route for doing this complicated process which happens alot
      Point3D      meterPoint = new Point3D(meterLoc.getEcefX(), meterLoc.getEcefY(), meterLoc.getEcefZ());
      Route        route      = Route.fromMessage(pluginServiceLocator.getRouteService().getCurrentRoute());
      RouteSegment meterSeg   = route.routeSegmentOfPoint(meterPoint, route.getSegments());                   // TODO optimize this
      // Get the point in location of the meter in segment frame
      Vector3 meterPointInSeg = meterSeg.getECEFToSegmentTransform().invert().apply(new Vector3(meterPoint.getX(), meterPoint.getY(), meterPoint.getZ()));
      double  segmentDTD      = route.lengthOfSegments(0, meterSeg.getDowntrackWaypoint().getWaypointId() - 1);
      double  meterDTD        = segmentDTD + meterPointInSeg.getX();
      double mergeDTD = meterDTD + mergeDTDFromMeter;

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
      plugin.setState(new PlanningState(plugin, log, pluginServiceLocator, rsuData));
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
