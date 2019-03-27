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
import java.util.concurrent.atomic.AtomicLong;

import cav_msgs.MobilityOperation;
import cav_msgs.MobilityRequest;
import cav_msgs.MobilityResponse;
import gov.dot.fhwa.saxton.carma.guidance.arbitrator.TrajectoryPlanningResponse;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.IComplexManeuver;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.IManeuver;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.ManeuverType;
import gov.dot.fhwa.saxton.carma.guidance.mobilityrouter.MobilityRequestResponse;
import gov.dot.fhwa.saxton.carma.guidance.plugins.PluginServiceLocator;
import gov.dot.fhwa.saxton.carma.guidance.trajectory.Trajectory;
import gov.dot.fhwa.saxton.carma.guidance.util.ILogger;
import gov.dot.fhwa.saxton.carma.rosutils.MobilityHelper;

/**
 * ExecutionState handles the execution of a cooperative merge for the CooperativeMergePlugin
 * Speed and steering commands received from an rsu are sent to the complex maneuver for execution.
 */
public class ExecutionState implements ICooperativeMergeState {

  protected static final double MIN_TARGET_SPEED = 0.5; // m/s - if traffic is heavy, a slow speed may be okay
  protected static final double MAX_TARGET_SPEED = 35.0; // m/s - a little over 75 mph
  protected static final double MIN_ACCEL = -2.0; // m/s^2 - trying to keep it smooth regardless of vehicle type
  protected static final double MAX_ACCEL = 2.0;  // m/s^2
  protected static final double MIN_TARGET_STEER = -1.0; //radians; currently not used since v2 steering is manual
  protected static final double MAX_TARGET_STEER = 1.0; //radians; currently not used since v2 steering is manual

  protected final CooperativeMergePlugin   plugin;
  protected final ILogger        log;
  protected final PluginServiceLocator pluginServiceLocator;
  protected final String OPERATION_PARAMS = "STATUS|METER_DIST:%.2f,MERGE_DIST:%.2f,SPEED:%.2f,LANE:%d";
  protected final String COMMAND_PARAM_TYPE = "COMMAND";
  protected final List<String> COMMAND_PARAM_KEYS = new ArrayList<>(Arrays.asList("SPEED", "ACCEL", "STEERING_ANGLE"));
  protected final String planId;
  protected final RampMeterData rampMeterData;
  protected final IComplexManeuver complexManeuver;
  protected boolean replanningForMerge = false;
  protected boolean executingMergeManeuver = false;
  protected AtomicLong lastCommandStamp = new AtomicLong(0);

  /**
   * Constructor
   * 
   * @param plugin The cooperative merge plugin
   * @param log The logger to use
   * @param pluginServiceLocator Provides access to vehicle data
   * @param rampMeterData The data on the rsu ramp meter providing commands to the vehicle
   * @param planId The id of the current plan being communicated with the rsu
   * @param complexManeuver The complex maneuver which is being executed
   */
  public ExecutionState(CooperativeMergePlugin plugin, ILogger log, PluginServiceLocator pluginServiceLocator,
    RampMeterData rampMeterData, String planId, IComplexManeuver complexManeuver) {

    this.plugin               = plugin;
    this.log                  = log;
    this.pluginServiceLocator = pluginServiceLocator;
    this.rampMeterData        = rampMeterData;
    this.planId               = planId;
    this.complexManeuver      = complexManeuver;
    // Set message stamp for timeout
    lastCommandStamp.set(System.currentTimeMillis());
  }
  
  @Override
  public TrajectoryPlanningResponse planTrajectory(Trajectory traj, double expectedEntrySpeed) {
    TrajectoryPlanningResponse tpr = new TrajectoryPlanningResponse();
    // We should not be planning in this state

    // If the trajectory we are asked to plan for is before the end of our complex maneuver an error must have occured
    // Log a warning and do not plan
    if (traj.getStartLocation() < complexManeuver.getEndDistance() - 0.5) {
      log.warn("Asked to plan trajectory before end of complex maneuver. Aborting rsu control");
      plugin.setState(this, new StandbyState(plugin, log, pluginServiceLocator));
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
    if (!msg.getHeader().getSenderId().equals(rampMeterData.getRsuId())
      || !msg.getHeader().getPlanId().equals(planId)) {
        return;
      }

    if (msg.getIsAccepted()) {
      log.warn("Unexpected MobilityResponse ACK received");
      return;
    }

    // This is a NACK. We need to abort and replan
    log.warn("NACK received from RSU emergency replanning plan id: " + msg.getHeader().getPlanId());
    // Return to standby state
    plugin.setState(this, new StandbyState(plugin, log, pluginServiceLocator));
    // Replan
    pluginServiceLocator.getArbitratorService().notifyTrajectoryFailure();
  }
  
  @Override
  public void onMobilityOperationMessage(MobilityOperation msg) {
    // Check if this response if for our proposed plan to merge
    if (!msg.getHeader().getSenderId().equals(rampMeterData.getRsuId())
    || !msg.getHeader().getPlanId().equals(planId)) {
      return;
    }

    // Extract params and validate
    // Expected String "COMMAND|SPEED:%.2f,ACCEL:%.2f,STEERING_ANGLE:%.2f" 
    // Extract params
    List<String> params;
    try {
      params = MobilityHelper.extractStrategyParams(msg.getStrategyParams(), COMMAND_PARAM_TYPE, COMMAND_PARAM_KEYS);
    } catch (IllegalArgumentException e) {
      log.error("Received operation message with bad params. Exception: " + e);
      pluginServiceLocator.getArbitratorService().notifyTrajectoryFailure();
      return;
    }

    // Params are valid so extract speed
    double targetSpeed = Double.parseDouble(params.get(0));
    double maxAccel = Double.parseDouble(params.get(1));
    double targetSteer = Double.parseDouble(params.get(2));

    //do a sanity check on the received values
    if (targetSpeed < MIN_TARGET_SPEED  ||  targetSpeed > MAX_TARGET_SPEED  ||
        maxAccel < MIN_ACCEL            ||  maxAccel > MAX_ACCEL  ||
        targetSteer < MIN_TARGET_STEER  ||  targetSteer > MAX_TARGET_STEER) {
      log.error("Received operation message with suspect strategy values. targetSpeed = " + targetSpeed +
                ", maxAccel = " + maxAccel + ", targetSteer = " + targetSteer);
      return;
    }

    targetSteer = 0;//TODO this is a temporary override as TO26 I-95 routes files do not match lane geometry

    plugin.getCooperativeMergeInputs().setCommands(targetSpeed, maxAccel, targetSteer);

    lastCommandStamp.set(System.currentTimeMillis());
  }

  /**
   * Helper function which publishes the current state of the vehicle for the RSU to process
   */
  private void publishOperationStatus() {
    MobilityOperation msg = plugin.getMobilityOperationPub().newMessage();
    // Build header
    msg.getHeader().setPlanId(planId);
    msg.getHeader().setRecipientId(rampMeterData.getRsuId());
    msg.getHeader().setSenderId(plugin.getVehicleId());
    msg.getHeader().setSenderBsmId("FFFFFFFF"); // TODO use real bsm id
    msg.getHeader().setTimestamp(System.currentTimeMillis());
    // Set contents
    msg.setStrategy(CooperativeMergePlugin.MOBILITY_STRATEGY);
    
    double currentDTD = pluginServiceLocator.getManeuverPlanner().getManeuverInputs().getDistanceFromRouteStart();
    String params = String.format(OPERATION_PARAMS, 
      rampMeterData.getRampMeterDTD() - currentDTD, 
      rampMeterData.getMergePointDTD() - currentDTD,
      pluginServiceLocator.getManeuverPlanner().getManeuverInputs().getCurrentSpeed(),
      pluginServiceLocator.getManeuverPlanner().getManeuverInputs().getCurrentLane()
    );

    msg.setStrategyParams(params);
    
    // Publish
    plugin.getMobilityOperationPub().publish(msg);
  }
  
  @Override
  public void loop() throws InterruptedException {
    // Check if this plugin is controlling the vehicle
    IManeuver currentManeuver = pluginServiceLocator.getArbitratorService().getCurrentlyExecutingManeuver(ManeuverType.COMPLEX);
    if (currentManeuver != null && currentManeuver.getPlanner().equals(plugin)) {
      // If in control publish status updates
      publishOperationStatus();
      executingMergeManeuver = true;

      // Check for comms timeout
      if (System.currentTimeMillis() - lastCommandStamp.get() > plugin.getCommsTimeoutMS()) {
        log.warn("RSU did not send operation message with command within timelimit");
        
        // Return to standby state and replan trajectory
        plugin.setState(this, new StandbyState(plugin, log, pluginServiceLocator));
        pluginServiceLocator.getArbitratorService().notifyTrajectoryFailure();
        return;
      }
    } else if (executingMergeManeuver) { // We were in control but no longer are
      executingMergeManeuver = false;
      // Return to standby state
      plugin.setState(this, new StandbyState(plugin, log, pluginServiceLocator));
      return;
    }
    // Sleep
    Thread.sleep(plugin.getUpdatePeriod());
  }
  
  @Override
  public String toString() {
    return this.getClass().getSimpleName();
  }
}
