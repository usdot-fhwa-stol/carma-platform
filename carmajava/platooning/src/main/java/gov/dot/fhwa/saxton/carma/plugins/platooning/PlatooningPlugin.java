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

package gov.dot.fhwa.saxton.carma.plugins.platooning;

import java.util.List;
import java.util.Arrays;
import java.util.concurrent.atomic.AtomicBoolean;

import cav_msgs.MobilityOperation;
import cav_msgs.MobilityRequest;
import cav_msgs.MobilityResponse;
import cav_msgs.PlatooningInfo;
import cav_msgs.SpeedAccel;
import gov.dot.fhwa.saxton.carma.guidance.arbitrator.TrajectoryPlanningResponse;
import gov.dot.fhwa.saxton.carma.guidance.conflictdetector.ConflictSpace;
import gov.dot.fhwa.saxton.carma.guidance.lightbar.ILightBarManager;
import gov.dot.fhwa.saxton.carma.guidance.lightbar.IndicatorStatus;
import gov.dot.fhwa.saxton.carma.guidance.lightbar.LightBarIndicator;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.IManeuverInputs;
import gov.dot.fhwa.saxton.carma.guidance.mobilityrouter.MobilityOperationHandler;
import gov.dot.fhwa.saxton.carma.guidance.mobilityrouter.MobilityRequestHandler;
import gov.dot.fhwa.saxton.carma.guidance.mobilityrouter.MobilityRequestResponse;
import gov.dot.fhwa.saxton.carma.guidance.mobilityrouter.MobilityResponseHandler;
import gov.dot.fhwa.saxton.carma.guidance.plugins.AbstractPlugin;
import gov.dot.fhwa.saxton.carma.guidance.plugins.IStrategicPlugin;
import gov.dot.fhwa.saxton.carma.guidance.plugins.PluginServiceLocator;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.IPublisher;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.ISubscriber;
import gov.dot.fhwa.saxton.carma.guidance.trajectory.Trajectory;

public class PlatooningPlugin extends AbstractPlugin
    implements IStrategicPlugin, MobilityOperationHandler, MobilityRequestHandler, MobilityResponseHandler {
    
    // TODO the plugin should use interface manager once rosjava multiple thread service call is fixed
    protected final String SPEED_CMD_CAPABILITY    = "/saxton_cav/drivers/srx_controller/control/cmd_speed";
    protected final String PLATOONING_FLAG         = "PLATOONING";
    protected final String MOBILITY_STRATEGY       = "Carma/Platooning";
    protected final String JOIN_AT_REAR_PARAMS     = "SIZE:%d,MAX_ACCEL:%.2f,DTD:%.2f";
    protected final String CANDIDATE_JOIN_PARAMS   = "DTD:%.2f";
    protected final String OPERATION_INFO_PARAMS   = "INFO|LEADER:%s,REAR_DTD:%.2f,SPEED:%.2f,SIZE:%d";
    protected final String OPERATION_STATUS_PARAMS = "STATUS|CMDSPEED:%.2f,DTD:%.2f,SPEED:%.2f";
    protected final String OPERATION_INFO_TYPE     = "INFO";
    protected final String OPERATION_STATUS_TYPE   = "STATUS";
    

    // initialize pubs/subs
    protected IPublisher<MobilityRequest>     mobilityRequestPublisher;
    protected IPublisher<MobilityOperation>   mobilityOperationPublisher;
    protected IPublisher<PlatooningInfo>      platooningInfoPublisher;
    protected ISubscriber<SpeedAccel>         cmdSpeedSub;

    // following parameters are for general platooning algorithm
    protected double maxAccel              = 2.5;
    protected double minimumManeuverLength = 15.0;
    protected double timeHeadway           = 1.8;
    protected double standStillGap         = 7.0;
    protected double kpPID                 = 1.5;
    protected double kiPID                 = 0.0;
    protected double kdPID                 = 0.1;
    protected double cmdSpeedMaxAdjustment = 10.0;
    
    // following parameters are for leader selection
    protected double lowerBoundary         = 1.65;
    protected double upperBoundary         = 1.75;
    protected double maxSpacing            = 2.0;
    protected double minSpacing            = 1.9;
    protected double minGap                = 10.0;
    protected double maxGap                = 15.0;
    protected int    algorithmType         = 1;
    
    // following parameters are for negotiation when a CAV want to join a platoon
    protected double maxJoinTime                    = 10.0;
    protected double desiredJoinTimeGap             = 4.0;
    protected double followerJoinTimeGap            = 3.0;
    protected double statusTimeoutFactor            = 2.5;
    protected int    statusIntervalLength           = 100;
    protected int    infoIntervalLength             = 3000;
    protected int    shortNegotiationTimeout        = 5000;
    protected int    longNegotiationTimeout         = 25000;
    protected int    maxPlatoonSize                 = 10;

    // platooning plug-in components
    protected IPlatooningState state                  = null;
    protected Thread           stateThread            = null;
    protected CommandGenerator commandGenerator       = null;
    protected Thread           commandGeneratorThread = null;
    protected PlatoonManager   platoonManager         = null;
    protected Thread           platoonManagerThread   = null;
    
    // initialize a lock for handle mobility messages
    protected Object sharedLock = new Object();

    // Light Bar Control
    protected final LightBarIndicator LIGHT_BAR_INDICATOR = LightBarIndicator.YELLOW;
    protected ILightBarManager lightBarManager;
    protected AtomicBoolean lostControlOfLights = new AtomicBoolean(false);
    protected final int LOOPS_PER_REQUEST = 10;
    protected int requestControlLoopsCount = 0;
    protected IndicatorStatus lastAttemptedIndicatorStatus = IndicatorStatus.OFF;
    
    // Control on MobilityRouter
    protected AtomicBoolean handleMobilityPath = new AtomicBoolean(true);

    public PlatooningPlugin(PluginServiceLocator pluginServiceLocator) {
        super(pluginServiceLocator);
        version.setName("Platooning Plugin");
        version.setMajorRevision(1);
        version.setIntermediateRevision(0);
        version.setMinorRevision(1);
    }

    @Override
    public void onInitialize() {
        log.info("Platooning plugin is initializing...");
        // initialize parameters of platooning plugin
        maxAccel                = pluginServiceLocator.getParameterSource().getDouble("~platooning_max_accel", 2.5);
        minimumManeuverLength   = pluginServiceLocator.getParameterSource().getDouble("~platooning_min_maneuver_length", 15.0);
        timeHeadway             = pluginServiceLocator.getParameterSource().getDouble("~platooning_desired_time_headway", 1.8);
        standStillGap           = pluginServiceLocator.getParameterSource().getDouble("~platooning_min_gap", 7.0);
        kpPID                   = pluginServiceLocator.getParameterSource().getDouble("~platooning_Kp", 1.5);
        kiPID                   = pluginServiceLocator.getParameterSource().getDouble("~platooning_Ki", 0.0);
        kdPID                   = pluginServiceLocator.getParameterSource().getDouble("~platooning_Kd", 0.1);
        cmdSpeedMaxAdjustment   = pluginServiceLocator.getParameterSource().getDouble("~platooning_cmd_max_adjustment", 10.0);
        statusTimeoutFactor     = pluginServiceLocator.getParameterSource().getDouble("~platooning_status_timeout_factor", 2.5);
        lowerBoundary           = pluginServiceLocator.getParameterSource().getDouble("~platooning_lower_boundary", 1.65);
        upperBoundary           = pluginServiceLocator.getParameterSource().getDouble("~platooning_upper_boundary", 1.75);
        maxSpacing              = pluginServiceLocator.getParameterSource().getDouble("~platooning_max_spacing", 2.0);
        minSpacing              = pluginServiceLocator.getParameterSource().getDouble("~platooning_min_spacing", 1.9);
        minGap                  = pluginServiceLocator.getParameterSource().getDouble("~platooning_min_gap", 10.0);
        maxGap                  = pluginServiceLocator.getParameterSource().getDouble("~platooning_max_gap", 15.0);
        statusIntervalLength    = pluginServiceLocator.getParameterSource().getInteger("~platooning_status_interval", 100);
        algorithmType           = pluginServiceLocator.getParameterSource().getInteger("~algorithm_type", 1);
        maxJoinTime             = pluginServiceLocator.getParameterSource().getDouble("~max_join_time", 10.0);
        desiredJoinTimeGap      = pluginServiceLocator.getParameterSource().getDouble("~desired_join_time_gap", 4.0);
        followerJoinTimeGap     = pluginServiceLocator.getParameterSource().getDouble("~follower_join_time_gap", 3.0);
        infoIntervalLength      = pluginServiceLocator.getParameterSource().getInteger("~operation_info_interval_length", 3000);
        shortNegotiationTimeout = pluginServiceLocator.getParameterSource().getInteger("~short_negotiation_timeout", 5000);
        longNegotiationTimeout  = pluginServiceLocator.getParameterSource().getInteger("~long_negotiation_timeout", 25000);
        maxPlatoonSize          = pluginServiceLocator.getParameterSource().getInteger("~max_platoon_size", 10);
        
        //log all loaded parameters
        log.debug("Load param maxAccel = " + maxAccel);
        log.debug("Load param minimumManeuverLength = " + minimumManeuverLength);
        log.debug("Load param timeHeadway = " + timeHeadway);
        log.debug("Load param standStillGap = " + standStillGap);
        log.debug("Load param for speed PID controller: [p = " + kpPID + ", i = " + kiPID + ", d = " + kdPID + "]");
        log.debug("Load param messageIntervalLength = " + statusIntervalLength);
        log.debug("Load param messageTimeoutFactor = " + statusTimeoutFactor);        
        log.debug("Load param lowerBoundary = " + lowerBoundary);        
        log.debug("Load param upperBoundary = " + upperBoundary);        
        log.debug("Load param maxSpacing = " + maxSpacing);        
        log.debug("Load param minSpacing = " + minSpacing);        
        log.debug("Load param minGap = " + minGap);        
        log.debug("Load param maxGap = " + maxGap);        
        log.debug("Load param algorithmType = " + algorithmType);
        log.debug("Load param maxJoinTime = " + maxJoinTime);
        log.debug("Load param desiredJoinTimeGap = " + desiredJoinTimeGap);
        log.debug("Load param infoIntervalLength = " + infoIntervalLength);
        log.debug("Load param shortNegotiationTimeout = " + shortNegotiationTimeout);
        log.debug("Load param longNegotiationTimeout = " + longNegotiationTimeout);
        log.debug("Load param maxPlatoonSize = " + maxPlatoonSize);
        log.debug("Load param followerJoinTimeGap = " + followerJoinTimeGap);
        log.debug("Load param cmdSpeedMaxAdjustment = " + cmdSpeedMaxAdjustment);
        // initialize necessary pubs/subs
        mobilityRequestPublisher   = pubSubService.getPublisherForTopic("outgoing_mobility_request", MobilityRequest._TYPE);
        mobilityOperationPublisher = pubSubService.getPublisherForTopic("outgoing_mobility_operation", MobilityOperation._TYPE);
        platooningInfoPublisher    = pubSubService.getPublisherForTopic("platooning_info", PlatooningInfo._TYPE);
        cmdSpeedSub                = pubSubService.getSubscriberForTopic(SPEED_CMD_CAPABILITY, SpeedAccel._TYPE);
        log.info("Platooning plugin is initialized.");
        // get light bar manager
        lightBarManager = pluginServiceLocator.getLightBarManager();
        // get control on mobility path capability
        AtomicBoolean tempCapability = pluginServiceLocator.getMobilityRouter().acquireDisableMobilityPathCapability();
        if(tempCapability != null) {
            this.handleMobilityPath = tempCapability;
            log.debug("Acquired control on mobility router for handling mobility path");
        } else {
            log.warn("Try to acquire control on mobility router for handling mobility path but failed");
        }
    }

    @Override
    public void onResume() {
        log.info("Platooning plugin resume to operate.");
        // register with MobilityRouter
        pluginServiceLocator.getMobilityRouter().registerMobilityRequestHandler(MOBILITY_STRATEGY, this);
        pluginServiceLocator.getMobilityRouter().registerMobilityResponseHandler(this);
        pluginServiceLocator.getMobilityRouter().registerMobilityOperationHandler(MOBILITY_STRATEGY, this);
        // reset plug-in's sub-components
        this.setState(new StandbyState(this, log, pluginServiceLocator));
        if(platoonManagerThread == null) {
            platoonManager       = new PlatoonManager(this, log, pluginServiceLocator);
            platoonManagerThread = new Thread(platoonManager);
            platoonManagerThread.setName("Platooning List Manager");
            platoonManagerThread.start();
            log.debug("Started platoonManagerThread");
        }
        if(commandGeneratorThread == null) {
            commandGenerator       = new CommandGenerator(this, log, pluginServiceLocator);
            commandGeneratorThread = new Thread(commandGenerator);
            commandGeneratorThread.setName("Platooning Command Generator");
            commandGeneratorThread.start();
            log.debug("Started commandGeneratorThread");
        }
        log.info("The current platooning plugin state is " + this.state.toString());
        this.setAvailability(true);
        // Take control of light bar indicator
        takeControlOfLightBar();
    }

    @Override
    public void onSuspend() {
        this.setAvailability(false);
        if(stateThread != null) {
            stateThread.interrupt();
            stateThread = null;
        }
        if(commandGeneratorThread != null) {
            commandGeneratorThread.interrupt();
            commandGeneratorThread = null;
        }
        if(platoonManagerThread != null) {
            platoonManagerThread.interrupt();
            platoonManagerThread = null;
        }
        log.info("Platooning plugin is suspended.");
        // Turn off lights and release control
        lightBarManager.setIndicator(LIGHT_BAR_INDICATOR, IndicatorStatus.OFF, this.getVersionInfo().componentName());
        lightBarManager.releaseControl(Arrays.asList(LIGHT_BAR_INDICATOR), this.getVersionInfo().componentName());
        // Release control on mobility router
        pluginServiceLocator.getMobilityRouter().releaseDisableMobilityPathCapability(handleMobilityPath);
    }
    
    @Override
    public void onTerminate() {
        // NO-OP
    }
    
    @Override
    public void loop() throws InterruptedException {
        // publish platooning information message for the usage of UI
        publishPlatooningInfo();
        // Request control of light bar if needed
        if (lostControlOfLights.get() == true) {
            if (requestControlLoopsCount == LOOPS_PER_REQUEST) {
                takeControlOfLightBar();
                requestControlLoopsCount = 0;
            }
        }
        Thread.sleep(statusIntervalLength);
    }

    @Override
    public TrajectoryPlanningResponse planTrajectory(Trajectory traj, double expectedEntrySpeed) {
        log.info("Plan Trajectory from " + traj.toString() + " in state " + state.toString());
        return this.state.planTrajectory(traj, expectedEntrySpeed);
    }
    
    @Override
    public void handleMobilityOperationMessage(MobilityOperation msg) {
        synchronized (this.sharedLock) {
            this.state.onMobilityOperationMessage(msg);
        }
    }
    
    @Override
    public MobilityRequestResponse handleMobilityRequestMessage(MobilityRequest msg, boolean hasConflict, ConflictSpace conflictSpace) {
        synchronized (this.sharedLock) {
            return this.state.onMobilityRequestMessgae(msg);
        }
    }
    
    @Override
    public void handleMobilityResponseMessage(MobilityResponse msg) {
        synchronized (this.sharedLock) {
            this.state.onMobilityResponseMessage(msg);
        }
    }
    
    // Set state for the current plug-in
    protected void setState(IPlatooningState state) {
        if(stateThread != null) {
            stateThread.interrupt();
            stateThread = null;
        }
        String previousState = this.state == null ? "NULL" : this.state.toString();
        log.info("Platooning plugin is changing from " + previousState + " state to " + state.toString() + " state");
        this.state = state;
        stateThread = new Thread(state);
        stateThread.start();
        log.debug("Started stateThread");
    }
    
    private void publishPlatooningInfo() {
        if (platooningInfoPublisher != null) {
            PlatooningInfo info = platooningInfoPublisher.newMessage();
            if(this.state instanceof StandbyState) {
                info.setState(PlatooningInfo.DISABLED);
            } else if(this.state instanceof LeaderState) {
                info.setState(platoonManager.getPlatooningSize() == 0 ? PlatooningInfo.SEARCHING : PlatooningInfo.LEADING);
            } else if(this.state instanceof LeaderWaitingState) {
                info.setState(PlatooningInfo.CONNECTING_TO_NEW_FOLLOWER);
            } else if(this.state instanceof CandidateFollowerState) {
                info.setState(PlatooningInfo.CONNECTING_TO_NEW_LEADER);
            } else if(this.state instanceof FollowerState) {
                info.setState(PlatooningInfo.FOLLOWING);
            }
            if(!(this.state instanceof StandbyState)) {
                info.setPlatoonId(this.platoonManager.currentPlatoonID);
                info.setSize((byte) this.platoonManager.platoonCurrentSize);
                info.setSizeLimit((byte) this.maxPlatoonSize);
                PlatoonMember currentLeader = this.platoonManager.getLeader();
                if(currentLeader == null) {
                    info.setLeaderId(pluginServiceLocator.getMobilityRouter().getHostMobilityId());
                    info.setLeaderDowntrackDistance((float) pluginServiceLocator.getRouteService().getCurrentDowntrackDistance());
                    info.setLeaderCmdSpeed((float) pluginServiceLocator.getManeuverPlanner().getManeuverInputs().getCurrentSpeed());
                    info.setHostPlatoonPosition((byte) 0); // platoon position is indexed as 1 based 
                    
                } else {
                    info.setLeaderId(currentLeader.staticId);
                    info.setLeaderDowntrackDistance((float) currentLeader.vehiclePosition);
                    info.setLeaderCmdSpeed((float) currentLeader.vehicleSpeed);
                    info.setHostPlatoonPosition((byte) platoonManager.getPlatooningSize()); // platoon position is indexed as 1 based
                }
                info.setHostCmdSpeed((float) (cmdSpeedSub.getLastMessage() != null ? cmdSpeedSub.getLastMessage().getSpeed() : 0.0));
                info.setDesiredGap((float) commandGenerator.desiredGap_);
            }
            platooningInfoPublisher.publish(info);
        }
    }

    /**
     * Helper function to acquire control of the light bar
     */
    private void takeControlOfLightBar() {
        List<LightBarIndicator> acquired = lightBarManager.requestControl(Arrays.asList(LIGHT_BAR_INDICATOR), this.getVersionInfo().componentName(),
            // Lost control of light call back
            (LightBarIndicator lostIndicator) -> {
                lostControlOfLights.set(true);
                log.info("Lost control of light bar indicator: " + LIGHT_BAR_INDICATOR);
        });
        // Check if the control request was successful. 
        if (acquired.contains(LIGHT_BAR_INDICATOR)) {
            lightBarManager.setIndicator(LIGHT_BAR_INDICATOR, lastAttemptedIndicatorStatus, this.getVersionInfo().componentName());
            lostControlOfLights.set(false);
            log.info("Got control of light bar indicator: " + LIGHT_BAR_INDICATOR);
        }
    }

    /**
     * Attempts to set the platooning controlled light bar indicators to the provided status
     * 
     * @param status the indicator status to set
     */
    protected void setLightBarStatus(IndicatorStatus status) {
        if (lightBarManager == null) {
            return;
        }
        lightBarManager.setIndicator(LIGHT_BAR_INDICATOR, status, this.getVersionInfo().componentName());
        lastAttemptedIndicatorStatus = status;
    }
    
    // The following getters will be helpful on doing unit tests, because it will let the plugin's
    // other components not depend on the actual platooning plug-in class. 
    protected CommandGenerator getCommandGenerator() {
        return commandGenerator;
    }

    protected PlatoonManager getPlatoonManager() {
        return platoonManager;
    }
    
    protected double getTimeHeadway() {
        return timeHeadway;
    }

    protected double getStandStillGap() {
        return standStillGap;
    }

    protected double getKpPID() {
        return kpPID;
    }

    protected double getKiPID() {
        return kiPID;
    }

    protected double getKdPID() {
        return kdPID;
    }
    
    protected double getMaxAccel() {
        return maxAccel;
    }
    
    protected IManeuverInputs getManeuverInputs() {
        return pluginServiceLocator.getManeuverPlanner().getManeuverInputs();
    }
    
    protected double getLowerBoundary() {
        return lowerBoundary;
    }

    protected double getUpperBoundary() {
        return upperBoundary;
    }

    protected double getMaxSpacing() {
        return maxSpacing;
    }

    protected double getMinSpacing() {
        return minSpacing;
    }

    protected double getMinGap() {
        return minGap;
    }

    protected double getMaxGap() {
        return maxGap;
    }
    
    protected int getAlgorithmType() {
        return algorithmType;
    }
    
    protected double getMaxJoinTime() {
        return maxJoinTime;
    }
    
    protected int getMaxPlatoonSize() {
        return maxPlatoonSize;
    }
    
    protected double getDesiredJoinTimeGap() {
        return desiredJoinTimeGap;
    }
    
    protected int getShortNegotiationTimeout() {
        return shortNegotiationTimeout;
    }

    protected int getLongNegotiationTimeout() {
        return longNegotiationTimeout;
    }
    
    protected int getOperationInfoIntervalLength() {
        return infoIntervalLength;
    }
    
    protected int getOperationUpdatesIntervalLength() {
        return statusIntervalLength;
    }
    
    protected double getMinimumManeuverLength() {
        return minimumManeuverLength;
    }
    
    protected double getOperationUpdatesTimeoutFactor() {
        return statusTimeoutFactor;
    }
    
    protected IPublisher<MobilityRequest> getMobilityRequestPublisher() {
        return mobilityRequestPublisher;
    }

    protected IPublisher<MobilityOperation> getMobilityOperationPublisher() {
        return mobilityOperationPublisher;
    }
    
    protected ISubscriber<SpeedAccel> getCmdSpeedSub() {
        return cmdSpeedSub;
    }
    
    protected double getFollowerJoinTimeGap() {
        return followerJoinTimeGap;
    }
    
    protected double getCmdSpeedMaxAdjustment() {
        return cmdSpeedMaxAdjustment;
    }
    
    protected AtomicBoolean getHandleMobilityPath() {
        return handleMobilityPath;
    }
    
    protected double getLastSpeedCmd() {
        return cmdSpeedSub.getLastMessage() == null ? 0.0 : cmdSpeedSub.getLastMessage().getSpeed();  
    }
}
