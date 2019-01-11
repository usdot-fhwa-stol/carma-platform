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

import java.util.concurrent.atomic.AtomicReference;

import cav_msgs.MobilityOperation;
import cav_msgs.MobilityRequest;
import cav_msgs.MobilityResponse;
import gov.dot.fhwa.saxton.carma.guidance.arbitrator.TrajectoryPlanningResponse;
import gov.dot.fhwa.saxton.carma.guidance.conflictdetector.ConflictSpace;
import gov.dot.fhwa.saxton.carma.guidance.mobilityrouter.MobilityOperationHandler;
import gov.dot.fhwa.saxton.carma.guidance.mobilityrouter.MobilityRequestHandler;
import gov.dot.fhwa.saxton.carma.guidance.mobilityrouter.MobilityRequestResponse;
import gov.dot.fhwa.saxton.carma.guidance.mobilityrouter.MobilityResponseHandler;
import gov.dot.fhwa.saxton.carma.guidance.plugins.AbstractPlugin;
import gov.dot.fhwa.saxton.carma.guidance.plugins.IStrategicPlugin;
import gov.dot.fhwa.saxton.carma.guidance.plugins.PluginServiceLocator;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.IPublisher;
import gov.dot.fhwa.saxton.carma.guidance.trajectory.Trajectory;

/**
 * Plugin implementing integration with a ramp metering RSU for cooperative merge application
 * <p>
 * Commmunicates via DSRC with a RSU functioning as a ramp meter. 
 * The RSU will send speed and steering commands to the vehicle until the merge is complete.
 */
public class CooperativeMergePlugin extends AbstractPlugin 
  implements IStrategicPlugin, MobilityRequestHandler, MobilityOperationHandler, MobilityResponseHandler {
  
  protected double minimumManeuverLength = 10.0; // m
  protected double maxAccel = 2.5; // m/s^2
  protected double lagTime = 0.1; // s
  protected IPublisher<MobilityRequest> mobilityRequestPub;
  protected IPublisher<MobilityOperation> mobilityOperationPub;
  protected IPublisher<MobilityResponse> mobilityResponsePub;
  
  protected AtomicReference<ICooperativeMergeState> state = new AtomicReference<>(null);

  protected CooperativeMergeInputs cooperativeMergeInputs;

  protected long commsTimeoutMS = 500; //ms
  protected long updatePeriod = 200; //ms

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
    pluginServiceLocator.getMobilityRouter().registerMobilityResponseHandler(this);
    
    cooperativeMergeInputs = new CooperativeMergeInputs(pluginServiceLocator.getManeuverPlanner().getManeuverInputs());
    lagTime = pluginServiceLocator.getManeuverPlanner().getManeuverInputs().getResponseLag();
    setState(null, new StandbyState(this, log, pluginServiceLocator));
  }

  @Override
  public void onInitialize() {
    double freq = pluginServiceLocator.getParameterSource().getDouble("~data_reporting_frequency", 5.0);
    updatePeriod = (long) (1000.0 / freq);

    commsTimeoutMS = pluginServiceLocator.getParameterSource().getInteger("~cooperative_merge_comms_timeout", 500);

    log.info("LoadedParam: data_reporting_frequency: " + freq + " Hz");
    log.info("LoadedParam: cooperative_merge_comms_timeout: " + commsTimeoutMS);

    mobilityRequestPub = pluginServiceLocator.getPubSubService().getPublisherForTopic("outgoing_mobility_request", MobilityRequest._TYPE);
    mobilityOperationPub = pluginServiceLocator.getPubSubService().getPublisherForTopic("outgoing_mobility_operation", MobilityOperation._TYPE);
    mobilityResponsePub = pluginServiceLocator.getPubSubService().getPublisherForTopic("outgoing_mobility_response", MobilityResponse._TYPE);
  }

  @Override
  public void onResume() {
    // Reset state
    synchronized(state) {
      setState(state.get(), new StandbyState(this, log, pluginServiceLocator));
    }
  }

  @Override
  public void loop() throws InterruptedException {
    state.get().loop();
  }

  @Override
  public void onSuspend() {
    // NO-OP
  }

  @Override
  public void onTerminate() {
    // NO-OP
  }

  @Override
  public TrajectoryPlanningResponse planTrajectory(Trajectory traj, double expectedStartSpeed) {
    return state.get().planTrajectory(traj, expectedStartSpeed);
  }

  @Override
  public void handleMobilityOperationMessage(MobilityOperation msg) {
      state.get().onMobilityOperationMessage(msg);
  }

  @Override
  public MobilityRequestResponse handleMobilityRequestMessage(MobilityRequest msg, boolean hasConflict,
      ConflictSpace conflictSpace) {

      return state.get().onMobilityRequestMessage(msg);
  }

  /**
   * Set the availability of this plugin for planning
   * 
   * @param availability The availability to set
   */
  protected void setAvailable(boolean availability) {
    this.setAvailability(availability);
  }

  /**
   * Sets the state of this plugin
   * The contact here is that the old state is always the state requesting the transition
   * This function is thread safe as long as the contract is met
   * 
   * @param callingState The state which is requesting the transition
   * @param newState The state to transition to
   */
  protected void setState(ICooperativeMergeState callingState, ICooperativeMergeState newState) {

    // Only perform the transition if we are still in the expected calling state
    if (state.compareAndSet(callingState, newState)) {
      log.info("Transitioned from old state: " + callingState + " to new state: " + newState); 
    }
  }


  /**
   * @return the vehicleId
   */
  public String getVehicleId() {
    return pluginServiceLocator.getMobilityRouter().getHostMobilityId();
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

  /**
   * @return the commsTimeoutMS
   */
  public long getCommsTimeoutMS() {
    return commsTimeoutMS;
  }

  /**
   * @return the updatePeriod
   */
  public long getUpdatePeriod() {
    return updatePeriod;
  }

  @Override
  public void handleMobilityResponseMessage(MobilityResponse msg) {
    state.get().onMobilityResponseMessage(msg);
  }
}
