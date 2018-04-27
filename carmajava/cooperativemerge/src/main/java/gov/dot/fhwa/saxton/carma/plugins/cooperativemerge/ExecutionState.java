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
import java.util.UUID;
import java.util.concurrent.atomic.AtomicBoolean;

import org.ros.internal.message.RawMessage;
import org.ros.rosjava_geometry.Vector3;

import cav_msgs.MobilityOperation;
import cav_msgs.MobilityRequest;
import cav_msgs.MobilityResponse;
import gov.dot.fhwa.saxton.carma.geometry.cartesian.Point3D;
import gov.dot.fhwa.saxton.carma.guidance.arbitrator.TrajectoryPlanningResponse;
import gov.dot.fhwa.saxton.carma.guidance.lightbar.IndicatorStatus;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.AccStrategyManager;
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
public class ExecutionState implements ICooperativeMergeState {
  
  protected static int LOOP_SLEEP_TIME   = 1000;
  
  protected CooperativeMergePlugin   plugin;
  protected ILogger        log;
  protected PluginServiceLocator pluginServiceLocator;
  protected String MERGE_REQUEST_PARAMS = "MERGE|MAX_ACCEL:%.2f,LAG:%.2f,DIST:%.2f";
  protected boolean replanningForMerge = false;
  protected Object replanningMutex = new Object();
  protected double rampMeterDTD;
  protected double mergePointDTD;
  
  public ExecutionState(CooperativeMergePlugin plugin, ILogger log, PluginServiceLocator pluginServiceLocator,
    double rampMeterDTD, double mergePointDTD) {

    this.plugin         = plugin;
    this.log          = log;
    this.pluginServiceLocator = pluginServiceLocator;
    this.rampMeterDTD = rampMeterDTD;
    this.mergePointDTD = mergePointDTD;

    // Notify meter of intention to merge
    plugin.setPlanId(UUID.randomUUID()); // Set the plan id

    MobilityRequest mergeRequest = plugin.mobilityRequestPub.newMessage();
    // Build Header
    mergeRequest.getHeader().setSenderId(plugin.getVehicleId());
    mergeRequest.getHeader().setRecipientId(plugin.getRsuId());
    mergeRequest.getHeader().setPlanId(plugin.getPlanId().toString());
    mergeRequest.getHeader().setSenderBsmId("FFFFFF"); // TODO use real BSM Id
    mergeRequest.getHeader().setTimestamp(System.currentTimeMillis());
    // Fill out request
    mergeRequest.setStrategy(plugin.MOBILITY_STRATEGY);
    
    String params = String.format(MERGE_REQUEST_PARAMS, plugin.getMaxAccel(), plugin.getLagTime(), 100.0); // TODO get distance to merge
    mergeRequest.setStrategyParams(params);
    
    mergeRequest.setLocation(mergeRequest.getLocation()); // TODO get the location
    mergeRequest.setExpiration(System.currentTimeMillis() + 500);
    
    plugin.getMobilityRequestPub().publish(mergeRequest);
  }
  
  @Override
  public TrajectoryPlanningResponse planTrajectory(Trajectory traj, double expectedEntrySpeed) {
    RouteService rs = pluginServiceLocator.getRouteService();
    TrajectoryPlanningResponse tpr = new TrajectoryPlanningResponse();
    // Check if the next trajectory includes a platooning window
    if(!rs.isAlgorithmEnabledInRange(traj.getStartLocation(), traj.getEndLocation(), CooperativeMergePlugin.COOPERATIVE_MERGE_FLAG)) {
      log.info("Asked to plan a trajectory without available window, ignoring...");
      return tpr;
    }

    // New Stuff
    synchronized (replanningMutex) {
      if (!replanningForMerge) {
        return tpr;
      }

      double currentDTD = pluginServiceLocator.getRouteService().getCurrentDowntrackDistance();
      
      // TODO add logic for if we start inside the circle
      if (currentDTD > rampMeterDTD) {
          log.warn("Did not change trajectory as we are already passed ramp meter point: " 
            + rampMeterDTD + " with downtrack: " + currentDTD);
          return tpr;
      }

      double lengthOfMergeRegion = 50; // TODO pass this in from rsu
      double complexManeuverSize = (mergePointDTD + lengthOfMergeRegion) - currentDTD;
      if (complexManeuverSize < plugin.getMinimumManeuverLength()) {
          log.warn(String.format("Failed to plan complex maneuver in trajectory: " + traj +
          ", downtrack: %.2f, merge point dtd: %.2f, length of merge: %.2f, min maneuver size: %.2f",
          currentDTD, mergePointDTD, lengthOfMergeRegion, plugin.getMinimumManeuverLength()));

          return tpr;
      }

      double start = traj.findEarliestLongitudinalWindowOfSize(complexManeuverSize);
      double end = start + complexManeuverSize;
      double currentSpeed = pluginServiceLocator.getManeuverPlanner().getManeuverInputs().getCurrentSpeed();

      // Check if the trajectory is long enough
      if (traj.getEndLocation() < end) {
          log.info("Planned Trajectory ended before merge completion. Requesting longer trajectory to " + end);
          
          tpr.requestLongerTrajectory(end);
          return tpr;
      }

      // TODO add logic for if we start inside the circle
      // If the first available planning window is before the meter point we need higher planning priority
      if (start > rampMeterDTD) {
          log.info("Requesting higher priority as current window is not sufficient. ramp meter point: " 
            + rampMeterDTD + " downtrack: " + currentDTD + " window start: " + start + " window size: " + complexManeuverSize);
          
          tpr.requestHigherPriority();
          return tpr;
      }

      // TODO we need some check for slow down feasibility

      // At this point we should have a valid window in which to plan

      // Build complex maneuver and add it to the trajectory
      CooperativeMergeManeuver mergeManeuver = new CooperativeMergeManeuver(
          plugin,
          plugin.getCooperativeMergeInputs(),
          pluginServiceLocator.getManeuverPlanner().getManeuverInputs(),
          pluginServiceLocator.getManeuverPlanner().getGuidanceCommands(),
          AccStrategyManager.newAccStrategy(),
          start, 
          end,
          0, 
          pluginServiceLocator.getRouteService().getSpeedLimitsInRange(start, end).first().getLimit());
      
      traj.setComplexManeuver(mergeManeuver);
      replanningForMerge = false;
    }

    return tpr;
  }

  @Override
  public MobilityRequestResponse onMobilityRequestMessage(MobilityRequest msg) {
    // No need to take actions for request messages at this time
    return MobilityRequestResponse.NO_RESPONSE;
  }
  
  @Override
  public void onMobilityResponseMessage(MobilityResponse msg) {
    // Check if this response if for our proposed plan to merge
    if (!msg.getHeader().getSenderId().equals(plugin.getRsuId())
      || !msg.getHeader().getPlanId().equals(plugin.getPlanId().toString())) {
        return;
      }

    synchronized (replanningMutex) {
      if (!replanningForMerge) {
        // If the request was accepted it is time to replan
        if (msg.getIsAccepted()) {
            plugin.setAvailable(true);
            replanningForMerge = true;
            pluginServiceLocator.getArbitratorService().requestNewPlan();
        }
      }
    }
  }
  
  @Override
  public void onMobilityOperationMessage(MobilityOperation msg) {
    // In standby state, it will ignore operation message since it is not actively operating
  }
  
  @Override
  public void run() {
    try {
      while(!Thread.currentThread().isInterrupted()) {
        Thread.sleep(LOOP_SLEEP_TIME);
      }
    } catch (InterruptedException e) {
      Thread.currentThread().interrupt();
    }
  }
  
  @Override
  public String toString() {
    return "StandbyState";
  }
}
