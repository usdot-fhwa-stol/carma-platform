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

import java.util.Arrays;
import java.util.UUID;
import java.util.concurrent.atomic.AtomicBoolean;

import cav_msgs.MobilityOperation;
import cav_msgs.MobilityRequest;
import cav_msgs.MobilityResponse;
import gov.dot.fhwa.saxton.carma.guidance.arbitrator.TrajectoryPlanningResponse;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.AccStrategyManager;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.IManeuverInputs;
import gov.dot.fhwa.saxton.carma.guidance.mobilityrouter.MobilityRequestResponse;
import gov.dot.fhwa.saxton.carma.guidance.plugins.PluginServiceLocator;
import gov.dot.fhwa.saxton.carma.guidance.trajectory.Trajectory;
import gov.dot.fhwa.saxton.carma.guidance.util.ILogger;
import gov.dot.fhwa.saxton.carma.guidance.util.RouteService;
import gov.dot.fhwa.saxton.carma.guidance.util.trajectoryconverter.ITrajectoryConverter;
import gov.dot.fhwa.saxton.carma.guidance.util.trajectoryconverter.RoutePointStamped;

/**
 * State responsible for planning a complex maneuver for the CooperativeMergePlugin
 */
public class PlanningState implements ICooperativeMergeState {
  
  protected final CooperativeMergePlugin   plugin;
  protected final ILogger        log;
  protected final PluginServiceLocator pluginServiceLocator;
  protected final RampMeterData rampMeterData;
  protected final long requestTime;
  protected final String planId;
  protected String MERGE_REQUEST_PARAMS      = "MERGE|MAX_ACCEL:%.2f,LAG:%.2f,DIST:%.2f";
  protected AtomicBoolean replanningForMerge = new AtomicBoolean(false);
  
  /**
   * Constructor
   * 
   * @param plugin The cooperative merge plugin
   * @param log The logger to use
   * @param pluginServiceLocator Used to access vehicle data
   * @param rampMeterData The data of the rsu being communicated with
   */
  public PlanningState(CooperativeMergePlugin plugin, ILogger log,
    PluginServiceLocator pluginServiceLocator, RampMeterData rampMeterData) {

    this.plugin               = plugin;
    this.log                  = log;
    this.pluginServiceLocator = pluginServiceLocator;
    this.rampMeterData        = rampMeterData;

    // Notify meter of intention to merge
    this.planId = UUID.randomUUID().toString(); // Set the plan id

    MobilityRequest mergeRequest = plugin.getMobilityRequestPub().newMessage();
    // Build Header
    mergeRequest.getHeader().setSenderId(plugin.getVehicleId());
    mergeRequest.getHeader().setRecipientId(rampMeterData.getRsuId());
    mergeRequest.getHeader().setPlanId(planId);
    mergeRequest.getHeader().setSenderBsmId("FFFFFFFF"); // TODO use real BSM Id
    mergeRequest.getHeader().setTimestamp(System.currentTimeMillis());
    // Fill out request
    mergeRequest.setStrategy(CooperativeMergePlugin.MOBILITY_STRATEGY);
    
    IManeuverInputs inputs = pluginServiceLocator.getManeuverPlanner().getManeuverInputs();
    RouteService rs = pluginServiceLocator.getRouteService();

    double currentDTD = rs.getCurrentDowntrackDistance();
    double currentCrosstrack = rs.getCurrentCrosstrackDistance();
    double currentSegmentDTD = rs.getCurrentSegmentDowntrack();
    int currentSegmentIdx = rs.getCurrentSegmentIndex();
    double distanceToMerge = rampMeterData.getMergePointDTD() - currentDTD;

    String params = String.format(MERGE_REQUEST_PARAMS, plugin.getMaxAccel(), plugin.getLagTime(), distanceToMerge); 
    mergeRequest.setStrategyParams(params);
    
    ITrajectoryConverter tc = pluginServiceLocator.getTrajectoryConverter();

    RoutePointStamped routePoint = new RoutePointStamped(currentDTD, currentCrosstrack,
     System.currentTimeMillis(), currentSegmentIdx, currentSegmentDTD);
    
    cav_msgs.Trajectory trajMsg = tc.pathToMessage(Arrays.asList(routePoint));
    mergeRequest.setLocation(trajMsg.getLocation()); 

    mergeRequest.setExpiration(System.currentTimeMillis() + 500);
    
    plugin.getMobilityRequestPub().publish(mergeRequest);
    this.requestTime = System.currentTimeMillis();
  }
  
  @Override
  public TrajectoryPlanningResponse planTrajectory(Trajectory traj, double expectedEntrySpeed) {
    RouteService rs = pluginServiceLocator.getRouteService();
    TrajectoryPlanningResponse tpr = new TrajectoryPlanningResponse();
    // Check if the next trajectory includes a platooning window
    if(!rs.isAlgorithmEnabledInRange(traj.getStartLocation(), traj.getEndLocation(), CooperativeMergePlugin.COOPERATIVE_MERGE_FLAG)) {
      log.info("Asked to plan a trajectory without available window, ignoring...");
      replanningForMerge.set(false);
      return tpr;
    }

    // New Stuff
    if (!replanningForMerge.get()) {
      return tpr;
    }

    double currentDTD = pluginServiceLocator.getRouteService().getCurrentDowntrackDistance();
    
    // TODO if needed add logic for if we start past the meter point
    if (currentDTD > rampMeterData.getRampMeterDTD()) {
      log.warn("Did not change trajectory as we are already passed ramp meter point: " 
        + rampMeterData.getRampMeterDTD() + " with downtrack: " + currentDTD);
      
      replanningForMerge.set(false);
      return tpr;
    }

    double complexManeuverSize = (rampMeterData.getMergePointDTD() + rampMeterData.getMergeLength()) - currentDTD;
    
    if (complexManeuverSize < plugin.getMinimumManeuverLength()) {
      log.warn(String.format("Failed to plan complex maneuver in trajectory: " + traj +
      ", downtrack: %.2f, merge point dtd: %.2f, length of merge: %.2f, min maneuver size: %.2f",
      currentDTD, rampMeterData.getMergePointDTD(), rampMeterData.getMergeLength(), plugin.getMinimumManeuverLength()));
      
      replanningForMerge.set(false);
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

    // TODO if needed add logic for if we start past the ramp meter point
    // If the first available planning window is before the meter point we need higher planning priority
    if (start > rampMeterData.getRampMeterDTD()) {
      log.info("Requesting higher priority as current window is not sufficient. ramp meter point: " 
        + rampMeterData.getRampMeterDTD() + " downtrack: " + currentDTD + " window start: " + start + " window size: " + complexManeuverSize);
      
      tpr.requestHigherPriority();
      return tpr;
    }

    // Evaluate if we will be able to stop before meter point
    // Uses kinematic equation v_f^2 = v_i^2 + 2ad
    // a = (v_f^2 - v_i^2) / 2d
    // v_f in this case = 0.0
    double deltaD = rampMeterData.getMergePointDTD() - currentDTD;
    double requiredAccel = Double.NEGATIVE_INFINITY;
    if (deltaD != 0) {
      requiredAccel = -(currentSpeed * currentSpeed) / (2 * deltaD);
    } else if (Math.abs(currentSpeed) < 0.1) {
      requiredAccel = 0;
    }

    double maxAccel = pluginServiceLocator.getManeuverPlanner().getManeuverInputs().getMaxAccelLimit();
    if (Math.abs(requiredAccel) > maxAccel) {
      log.warn("Cannot slow down in the needed space to stop. calculatedAccel: " + requiredAccel);
      return tpr;
    }
    

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
      rs.getSpeedLimitsInRange(start, end).first().getLimit());

    traj.setComplexManeuver(mergeManeuver);
    plugin.setState(new ExecutionState(plugin, log, pluginServiceLocator, rampMeterData, planId, mergeManeuver));
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
    if (!msg.getHeader().getSenderId().equals(rampMeterData.getRsuId())
      || !msg.getHeader().getPlanId().equals(planId)) {
        return;
      }

    synchronized (replanningForMerge) {
      if (!replanningForMerge.get()) {
        // If the request was accepted it is time to replan
        if (msg.getIsAccepted()) {
            plugin.setAvailable(true);
            replanningForMerge.set(true);
            pluginServiceLocator.getArbitratorService().requestNewPlan();
        }
      }
    }
  }
  
  @Override
  public void onMobilityOperationMessage(MobilityOperation msg) {
    // In standby state, it will ignore operation message since it is not actively operating
  }

  /**
   * Helper function to publish a nack message to the rsu for the current plan
   */
  private void publishNack() {
    MobilityResponse response = plugin.mobilityResponsePub.newMessage();

    response.getHeader().setSenderId(plugin.vehicleId);
    response.getHeader().setSenderBsmId("FFFFFFFF"); // TODO use real bsm id
    response.getHeader().setRecipientId(rampMeterData.getRsuId());
    response.getHeader().setPlanId(this.planId);
    response.getHeader().setTimestamp(System.currentTimeMillis());
    response.setIsAccepted(false);
  }
  
  @Override
  public void loop() throws InterruptedException {
    if (System.currentTimeMillis() - this.requestTime > plugin.getCommsTimeoutMS()) {
      log.warn("RSU did not accept request to merge within timelimit");
      publishNack();
      plugin.setState(new StandbyState(plugin, log, pluginServiceLocator));
      return;
    }
    Thread.sleep(plugin.getUpdatePeriod());
  }
  
  @Override
  public String toString() {
    return this.getClass().getSimpleName();
  }
}
