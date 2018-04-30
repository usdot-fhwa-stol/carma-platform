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

import java.util.List;

import org.ros.internal.message.RawMessage;
import org.ros.rosjava_geometry.Vector3;

import cav_msgs.MobilityOperation;
import cav_msgs.MobilityRequest;
import cav_msgs.MobilityResponse;
import gov.dot.fhwa.saxton.carma.geometry.cartesian.Point3D;
import gov.dot.fhwa.saxton.carma.guidance.arbitrator.TrajectoryPlanningResponse;
import gov.dot.fhwa.saxton.carma.guidance.lightbar.IndicatorStatus;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.SlowDown;
import gov.dot.fhwa.saxton.carma.guidance.mobilityrouter.MobilityRequestResponse;
import gov.dot.fhwa.saxton.carma.guidance.plugins.PluginServiceLocator;
import gov.dot.fhwa.saxton.carma.guidance.trajectory.Trajectory;
import gov.dot.fhwa.saxton.carma.guidance.util.ILogger;
import gov.dot.fhwa.saxton.carma.guidance.util.RouteService;
import gov.dot.fhwa.saxton.carma.route.Route;
import gov.dot.fhwa.saxton.carma.route.RouteSegment;
import std_msgs.Header;

/**
 * TODO
 * The StandbyState is a state when the platooning algorithm is current disabled on the route.
 * It will transit to SingleVehiclePlatoonState when it knows the algorithm will be enabled in the next trajectory.
 * In this state, the plug-in will not insert any maneuvers into a trajectory and will ignore all negotiation messages.
 */
public class StandbyState implements ICooperativeMergeState {
  
  protected static int LOOP_SLEEP_TIME   = 1000;
  
  protected CooperativeMergePlugin   plugin;
  protected ILogger        log;
  protected PluginServiceLocator pluginServiceLocator;
  
  public StandbyState(CooperativeMergePlugin plugin, ILogger log, PluginServiceLocator pluginServiceLocator) {
    this.plugin         = plugin;
    this.log          = log;
    this.pluginServiceLocator = pluginServiceLocator;
  }
  
  @Override
  public TrajectoryPlanningResponse planTrajectory(Trajectory traj, double expectedEntrySpeed) {
    RouteService rs = pluginServiceLocator.getRouteService();
    TrajectoryPlanningResponse tpr = new TrajectoryPlanningResponse();
    // Check if the next trajectory includes a platooning window
    if(!rs.isAlgorithmEnabledInRange(traj.getStartLocation(), traj.getEndLocation(), CooperativeMergePlugin.COOPERATIVE_MERGE_FLAG)) {
      log.info("In standby state, asked to plan a trajectory without available window, ignoring...");
    }
    
    return tpr;
  }

  @Override
  public MobilityRequestResponse onMobilityRequestMessage(MobilityRequest msg) {
    // TODO cache meter locations to improve performance
    // In standby state, the plugin waits to receive a message from a ramp metering rsu
    // Parse Strategy Params
    String expectedString = "INFO|RADIUS:%.2f"; // TODO add distance to merge point
    double mergeDTD = 0; // TODO
    final String RADIUS_PARAM = "RADIUS:";
    final String INFO_TYPE = "INFO";
    String paramsString = msg.getStrategyParams();
    String[] paramsParts = paramsString.split("\\|");
    String typeString = paramsParts[0];
    String dataString = paramsParts[1];
    String[] dataParts = dataString.split(":");
    String dataType = dataParts[0];
    // Validate Params
    if (!typeString.equals(INFO_TYPE) || !dataParts.equals(RADIUS_PARAM)) {
      log.info("Received mobility request with invalid params for state. Params: " + paramsString);
      return MobilityRequestResponse.NO_RESPONSE;
    }

    String senderId = msg.getHeader().getSenderId();

    double meterRadius = Double.parseDouble(dataParts[1]);

    cav_msgs.LocationECEF meterLoc = msg.getLocation();
    // TODO Probably would be good to add a function to route for doing this complicated process
    Point3D meterPoint = new Point3D(meterLoc.getEcefX(), meterLoc.getEcefY(), meterLoc.getEcefZ());
    Route route = Route.fromMessage(pluginServiceLocator.getRouteService().getCurrentRoute());
    RouteSegment meterSeg = route.routeSegmentOfPoint(meterPoint, route.getSegments()); // TODO optimize this
    // Get the point in location of the meter in segment frame
    Vector3 meterPointInSeg = meterSeg.getECEFToSegmentTransform().invert().apply(new Vector3(meterPoint.getX(), meterPoint.getY(), meterPoint.getZ()));
    double segmentDTD = route.lengthOfSegments(0, meterSeg.getDowntrackWaypoint().getWaypointId() - 1);
    double meterDTD = segmentDTD + meterPointInSeg.getX();


    double currentDTD = pluginServiceLocator.getRouteService().getCurrentDowntrackDistance();

    if (Math.abs(meterDTD - currentDTD) < meterRadius) {
      // Change to planning state
      plugin.setState(new PlanningState(plugin, log, pluginServiceLocator, meterDTD, mergeDTD));
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
