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

import cav_msgs.MobilityOperation;
import cav_msgs.MobilityRequest;
import cav_msgs.MobilityResponse;
import gov.dot.fhwa.saxton.carma.guidance.arbitrator.TrajectoryPlanningResponse;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.IManeuver;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.ManeuverType;
import gov.dot.fhwa.saxton.carma.guidance.mobilityrouter.MobilityRequestResponse;
import gov.dot.fhwa.saxton.carma.guidance.plugins.PluginServiceLocator;
import gov.dot.fhwa.saxton.carma.guidance.trajectory.Trajectory;
import gov.dot.fhwa.saxton.carma.guidance.util.ILogger;

/**
 * TODO
 * The StandbyState is a state when the platooning algorithm is current disabled on the route.
 * It will transit to SingleVehiclePlatoonState when it knows the algorithm will be enabled in the next trajectory.
 * In this state, the plug-in will not insert any maneuvers into a trajectory and will ignore all negotiation messages.
 */
public class ExecutionState implements ICooperativeMergeState {
  
  protected static int LOOP_SLEEP_TIME   = 500;
  
  protected CooperativeMergePlugin   plugin;
  protected ILogger        log;
  protected PluginServiceLocator pluginServiceLocator;
  protected String OPERATION_PARAMS = "STATUS|METER_DIST:%.2f,MERGE_DIST:%.2f,SPEED:%.2f";
  protected boolean replanningForMerge = false;
  protected Object replanningMutex = new Object();
  protected double rampMeterDTD;
  protected double mergePointDTD;
  protected boolean executingMergeManeuver = false;
  
  public ExecutionState(CooperativeMergePlugin plugin, ILogger log, PluginServiceLocator pluginServiceLocator,
    double rampMeterDTD, double mergePointDTD) {

    this.plugin         = plugin;
    this.log          = log;
    this.pluginServiceLocator = pluginServiceLocator;
    this.rampMeterDTD = rampMeterDTD;
    this.mergePointDTD = mergePointDTD;
  }
  
  @Override
  public TrajectoryPlanningResponse planTrajectory(Trajectory traj, double expectedEntrySpeed) {
    TrajectoryPlanningResponse tpr = new TrajectoryPlanningResponse();
    //TODO log and warn if planning over complex maneuver
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

    if (msg.getIsAccepted()) {
      log.warn("Unexpected MobilityResponse ACK received");
      return;
    }

    // This is a NACK. We need to abort and replan
    // TODO do we need more notifications here for the UI maybe?
    log.warn("NACK received from RSU emergency replanning plan id: " + msg.getHeader().getPlanId());
    pluginServiceLocator.getArbitratorService().notifyTrajectoryFailure();
  }
  
  @Override
  public void onMobilityOperationMessage(MobilityOperation msg) {
    // Check if this response if for our proposed plan to merge
    if (!msg.getHeader().getSenderId().equals(plugin.getRsuId())
    || !msg.getHeader().getPlanId().equals(plugin.getPlanId().toString())) {
      return;
    }

    // Extract params and validate
    // TODO is it better to fail fast here?
    try {
      String expectedString = "COMMAND|SPEED:%.2f,STEERING_ANGLE:%.2f"; 
      final String SPEED_PARAM = "SPEED";
      final String STEERING_PARAM = "STEERING_ANGLE";
      final String TYPE = "COMMAND";
      String paramsString = msg.getStrategyParams();
      String[] paramsParts = paramsString.split("\\|");
      String typeString = paramsParts[0];
      String dataString = paramsParts[1];
      String[] dataParts = dataString.split(",");
      String speedPart = dataParts[0];
      String steeringPart = dataParts[1];
      String[] speedParts = speedPart.split(":");
      String[] steeringParts = steeringPart.split(":");

      if (!typeString.equals(TYPE) || !speedParts[0].equals(SPEED_PARAM)
          || !steeringParts[0].equals(STEERING_PARAM)) {
        log.error("MobilityOperationMessage received from RSU with bad prams: " + paramsString);
        pluginServiceLocator.getArbitratorService().notifyTrajectoryFailure();
        return;
      }

      // Params are valid so extract speed
      double targetSpeed = Double.parseDouble(speedParts[1]);
      double targetSteer = Double.parseDouble(steeringParts[1]);

      plugin.getCooperativeMergeInputs().setSpeedCommand(targetSpeed);
      plugin.getCooperativeMergeInputs().setSteeringCommand(targetSteer);

    } catch (IndexOutOfBoundsException e) {
      log.error("MobilityOperationMessage received from RSU with bad prams: " +  msg.getStrategyParams());
      pluginServiceLocator.getArbitratorService().notifyTrajectoryFailure();
      return;
    }
  }

  /**
   * Helper function which publishes the current state of the vehicle for the RSU to process
   */
  private void publishOperationStatus() {
    MobilityOperation msg = plugin.getMobilityOperationPub().newMessage();
    // Build header
    msg.getHeader().setPlanId(plugin.getPlanId().toString());
    msg.getHeader().setRecipientId(plugin.getRsuId());
    msg.getHeader().setSenderId(plugin.getVehicleId());
    msg.getHeader().setSenderBsmId("FFFFFFF"); // TODO use real bsm id
    msg.getHeader().setTimestamp(System.currentTimeMillis());
    // Set contents
    msg.setStrategy(CooperativeMergePlugin.MOBILITY_STRATEGY);
    
    double currentDTD = pluginServiceLocator.getManeuverPlanner().getManeuverInputs().getDistanceFromRouteStart();
    String params = String.format(OPERATION_PARAMS, 
      rampMeterDTD - currentDTD, 
      mergePointDTD - currentDTD,
      pluginServiceLocator.getManeuverPlanner().getManeuverInputs().getCurrentSpeed()
    );

    msg.setStrategyParams(params);
    
    // Publish
    plugin.getMobilityOperationPub().publish(msg);
  }
  
  @Override
  public void loop() throws InterruptedException {
    // Check if this plugin is controlling the vehicle
    IManeuver currentManeuver = pluginServiceLocator.getArbitratorService().getCurrentlyExecutingManeuver(ManeuverType.LONGITUDINAL);
    if (currentManeuver.getPlanner().equals(plugin)) {
      // If in control publish status updates
      publishOperationStatus();
      executingMergeManeuver = true;
    } else if (executingMergeManeuver) { // We were in control but no longer are
      executingMergeManeuver = false;
      // Return to standby state
      plugin.setState(new StandbyState(plugin, log, pluginServiceLocator));
    }
    // Sleep
    Thread.sleep(LOOP_SLEEP_TIME);
  }
  
  @Override
  public String toString() {
    return this.getClass().getSimpleName();
  }
}
