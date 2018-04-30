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

import java.util.UUID;

import cav_msgs.MobilityOperation;
import cav_msgs.MobilityRequest;
import cav_msgs.MobilityResponse;
import gov.dot.fhwa.saxton.carma.guidance.arbitrator.TrajectoryPlanningResponse;
import gov.dot.fhwa.saxton.carma.guidance.conflictdetector.ConflictSpace;
import gov.dot.fhwa.saxton.carma.guidance.mobilityrouter.MobilityOperationHandler;
import gov.dot.fhwa.saxton.carma.guidance.mobilityrouter.MobilityRequestHandler;
import gov.dot.fhwa.saxton.carma.guidance.mobilityrouter.MobilityRequestResponse;
import gov.dot.fhwa.saxton.carma.guidance.plugins.AbstractPlugin;
import gov.dot.fhwa.saxton.carma.guidance.plugins.IStrategicPlugin;
import gov.dot.fhwa.saxton.carma.guidance.plugins.PluginServiceLocator;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.IPublisher;
import gov.dot.fhwa.saxton.carma.guidance.trajectory.Trajectory;

/**
 * Plugin implementing integration withe STOL I TO 22 Infrastructure Server
 * <p>
 * Commmunicates via the internet with the Infrastructure Server to report vehicle
 * state and receive speed commands as may relate to whatever algorithm the server
 * is configured to run with.
 *///ICooperativeMergeInputs,
public class CooperativeMergePlugin extends AbstractPlugin implements IStrategicPlugin, MobilityRequestHandler, MobilityOperationHandler {
  protected String vehicleId = "";
  protected String rsuId = null;
  protected UUID planId = null;
  protected double minimumManeuverLength = 10.0;
  protected double maxAccel = 2.0;
  protected double lagTime = 0.1;
  protected long maneuverTimeout = 3000; //ms
  protected IPublisher<MobilityRequest> mobilityRequestPub;
  protected IPublisher<MobilityOperation> mobilityOperationPub;
  protected IPublisher<MobilityResponse> mobilityResponsePub;

  protected double meterDTD = 0;
  
  protected ICooperativeMergeState state = null;

  protected CooperativeMergeInputs cooperativeMergeInputs;

  // protected StatusUpdater statusUpdater = null;
  // protected Thread statusUpdaterThread = null;

  // protected CommandReceiver commandReceiver = null;
  // protected Thread commandReceiverThread = null;

  // protected SessionManager sessionManager;
  // protected VehicleDataManager vehicleDataManager;
  // protected LocalDateTime lastUpdateTime = LocalDateTime.now();
  // protected RestTemplate restClient;

  public static final String COOPERATIVE_MERGE_FLAG = "COOPERATIVE_MERGE";
  public static final String MOBILITY_STRATEGY = "Carma/CooperativeMerge";

  public CooperativeMergePlugin(PluginServiceLocator psl) {
    super(psl);
    version.setName("Cooperative Merge Plugin");
    version.setMajorRevision(1);
    version.setIntermediateRevision(0);
    version.setMinorRevision(0);
    // Register Mobility Message Callbacks
    pluginServiceLocator.getMobilityRouter().registerMobilityRequestHandler(MOBILITY_STRATEGY, this);
    pluginServiceLocator.getMobilityRouter().registerMobilityOperationHandler(MOBILITY_STRATEGY, this);
    maxAccel = 2.5; // TODO get this from somewhere
    cooperativeMergeInputs = new CooperativeMergeInputs(maxAccel);
    lagTime = pluginServiceLocator.getManeuverPlanner().getManeuverInputs().getResponseLag();
    //restClient = new RestTemplate();
  }

  @Override
  public void onInitialize() {
    vehicleId = pluginServiceLocator.getParameterSource().getString("vehicle_id");
    double freq = pluginServiceLocator.getParameterSource().getDouble("~data_reporting_frequency", 1.0);
    timestepDuration = (long) (1000.0 / freq);

    maneuverTimeout = pluginServiceLocator.getParameterSource().getInteger("~cooperative_merge_maneuver_timeout", 3000);

    log.info("LoadedParam: vehicle_id: " + vehicleId);
    log.info("LoadedParam: data_reporting_frequency: " + freq);
    log.info("LoadedParam: cooperative_merge_maneuver_timeout: " + maneuverTimeout);

    mobilityRequestPub = pluginServiceLocator.getPubSubService().getPublisherForTopic("outgoing_mobility_request", MobilityRequest._TYPE);
    mobilityRequestPub = pluginServiceLocator.getPubSubService().getPublisherForTopic("outgoing_mobility_operation", MobilityRequest._TYPE);
  }

  @Override
  public void onResume() {

    // if (statusUpdaterThread == null && statusUpdater == null) {
    //   statusUpdater = new StatusUpdater(serverUrl, sessionManager.getServerSessionId(), restClient, timestepDuration,
    //       vehicleDataManager);
    //   statusUpdaterThread = new Thread(statusUpdater);
    //   statusUpdaterThread.setName("SpeedHarm Status Updater");
    //   statusUpdaterThread.start();
    // }

    // if (commandReceiverThread == null && commandReceiver == null) {
    //   commandReceiver = new CommandReceiver(serverUrl, sessionManager.getServerSessionId(), restClient);
    //   commandReceiverThread = new Thread(commandReceiver);
    //   commandReceiverThread.setName("SpeedHarm Command Receiver");
    //   commandReceiverThread.start();
    // }
  }

  @Override
  public void loop() throws InterruptedException {

    if (state == null) {
      log.warn("Requested to loop but state was null");
      return;
    }

    state.loop();
    // long tsStart = System.currentTimeMillis();
    // if (statusUpdater.lastUpdateTime != null) {
    //   // If we've successfully communicated with the server recently, signal our availability
    //   java.time.Duration timeSinceLastUpdate = java.time.Duration.between(statusUpdater.lastUpdateTime,
    //       LocalDateTime.now());
    //   if (timeSinceLastUpdate.toMillis() < 3 * timestepDuration) {
    //     setAvailability(true);
    //   } else {
    //     setAvailability(false);
    //   }
    // }

    // vehicleDataManager.setManeuverRunning(pluginServiceLocator.getArbitratorService()
    //     .getCurrentlyExecutingManeuver(ManeuverType.COMPLEX) instanceof SpeedHarmonizationManeuver);

    // long tsEnd = System.currentTimeMillis();
    // long sleepDuration = Math.max(100 - (tsEnd - tsStart), 0);
    // Thread.sleep(sleepDuration);
  }

  @Override
  public void onSuspend() {
    // if (statusUpdaterThread != null && statusUpdater != null) {
    //   statusUpdaterThread.interrupt();
    //   statusUpdaterThread = null;
    //   statusUpdater = null;
    // }

    // if (commandReceiverThread != null && commandReceiver != null) {
    //   commandReceiverThread.interrupt();
    //   commandReceiverThread = null;
    //   commandReceiver = null;
    // }

    // sessionManager.endVehicleSession();
  }

  @Override
  public void onTerminate() {
    // NO-OP
  }

  @Override
  public TrajectoryPlanningResponse planTrajectory(Trajectory traj, double expectedStartSpeed) {
    if (state == null) {
      log.warn("Requested to plan trajectory but state was null");
      return new TrajectoryPlanningResponse();
    }
    return state.planTrajectory(traj, expectedStartSpeed);
  }

@Override // TODO Synchronize
public void handleMobilityOperationMessage(MobilityOperation msg) {
  if (state == null) {
    log.warn("Requested to handle mobility operation message but state was null");
    return;
  }
  state.onMobilityOperationMessage(msg);
}

@Override // TODO Synchronize
public MobilityRequestResponse handleMobilityRequestMessage(MobilityRequest msg, boolean hasConflict,
    ConflictSpace conflictSpace) {
  if (state == null) {
    log.warn("Requested to handle mobility request message but state was null");
    return MobilityRequestResponse.NO_RESPONSE;
  }
  return state.onMobilityRequestMessage(msg);
}

  // @Override
  // public double getSpeedCommand() {
  //   // if (commandReceiver.getLastCommand() != null) {
  //   //   log.info("Using received command");
  //   //   return commandReceiver.getLastCommand().getSpeed();
  //   // } else {
  //   //   log.info("Using previous vehicle speed");
  //   //   return vehicleDataManager.getSpeed();
  //   // }
  // }

  // @Override
  // public double getMaxAccelLimit() {
  //   return Math.min(Math.abs(maxAccel), 2.5);
  // }

  // @Override
  // public Duration getTimeSinceLastUpdate() {
  //   // if (commandReceiver != null && commandReceiver.getLastCommand() != null) {
  //   //   LocalDateTime now = LocalDateTime.now();
  //   //   long millis = java.time.Duration.between(commandReceiver.getLastCommand().getTimestamp(), now).toMillis();
  //   //   return Duration.fromMillis(millis);
  //   // } else {
  //   //   return Duration.fromMillis(0);
  //   // }
  // }

  protected void setAvailable(boolean availability) {
    this.setAvailability(availability);
  }

  protected void setState(ICooperativeMergeState newState) {
    log.info("Transitioned from old state: " + state + " to new state: " + newState);
    state = newState;
  }

  /**
   * @return the vehicleId
   */
  public String getVehicleId() {
    return vehicleId;
  }

  /**
   * @return the rsuId
   */
  public String getRsuId() {
    return rsuId;
  }

  /**
   * @return the planId
   */
  public UUID getPlanId() {
    return planId;
  }

  /**
   * @param planId the planId to set
   */
  public void setPlanId(UUID planId) {
    this.planId = planId;
  }

  /**
   * @return the maxAccel
   */
  public double getMaxAccel() {
    return maxAccel;
  }

  /**
   * @return the lagTime
   */
  public double getLagTime() {
    return lagTime;
  }

  /**
   * @return the mobilityOperationPub
   */
  public IPublisher<MobilityOperation> getMobilityOperationPub() {
    return mobilityOperationPub;
  }

  /**
   * @return the mobilityRequestPub
   */
  public IPublisher<MobilityRequest> getMobilityRequestPub() {
    return mobilityRequestPub;
  }

  /**
   * @return the mobilityResponsePub
   */
  public IPublisher<MobilityResponse> getMobilityResponsePub() {
    return mobilityResponsePub;
  }

  /**
   * @return the minimumManeuverLength
   */
  public double getMinimumManeuverLength() {
    return minimumManeuverLength;
  }
  
  /**
   * @return the cooperativeMergeInputs
   */
  public CooperativeMergeInputs getCooperativeMergeInputs() {
    return cooperativeMergeInputs;
  }

}