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

import java.util.UUID;

import cav_msgs.MobilityIntro;
import cav_msgs.NewPlan;
import cav_msgs.SpeedAccel;
import gov.dot.fhwa.saxton.carma.guidance.arbitrator.TrajectoryPlanningResponse;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.IManeuverInputs;
import gov.dot.fhwa.saxton.carma.guidance.plugins.AbstractPlugin;
import gov.dot.fhwa.saxton.carma.guidance.plugins.IStrategicPlugin;
import gov.dot.fhwa.saxton.carma.guidance.plugins.PluginServiceLocator;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.IPublisher;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.ISubscriber;
import gov.dot.fhwa.saxton.carma.guidance.trajectory.Trajectory;

public class PlatooningPlugin extends AbstractPlugin implements IStrategicPlugin {

    protected final String PLATOONING_FLAG = "PLATOONING";
    // TODO the plugin should use interface manager once rosjava multi-thread service call is fixed
    protected final String SPEED_CMD_CAPABILITY = "/saxton_cav/drivers/srx_controller/control/cmd_speed";
    
    protected IPlatooningState state;
    protected IPublisher<MobilityIntro> mobilityIntroPublisher;
    protected ISubscriber<NewPlan> newPlanSub;
    protected ISubscriber<SpeedAccel> cmdSpeedSub;
    protected IManeuverInputs maneuverInputs;
    
    protected double maxAccel = 2.5;
    protected double minimumManeuverLength = 15.0;
    protected double timeHeadway = 1.8;
    protected double standStillGap = 7.0;
    protected double kpPID = 1.5;
    protected double kiPID = 0.0;
    protected double kdPID = 0.1;
    protected int messageIntervalLength = 500;
    protected int messageTimeout = 750;
    
    protected CommandGenerator commandGenerator = null;
    protected Thread commandGeneratorThread = null;
    protected PlatoonManager platoonManager = null;
    protected Thread platoonManagerThread = null;
    
    protected final String STATIC_ID;
    
    public PlatooningPlugin(PluginServiceLocator pluginServiceLocator) {
        super(pluginServiceLocator);
        version.setName("CACC Platooning Plugin");
        version.setMajorRevision(1);
        version.setIntermediateRevision(0);
        version.setMinorRevision(0);
        STATIC_ID = UUID.randomUUID().toString();
    }

    @Override
    public void onInitialize() {
        log.info("CACC platooning pulgin is initializing...");
        // initialize parameters of platooning plugin
        maxAccel = pluginServiceLocator.getParameterSource().getDouble("~platooning_max_accel", 2.5);
        log.debug("Load param maxAccel = " + maxAccel);
        minimumManeuverLength = pluginServiceLocator.getParameterSource().getDouble("~platooning_min_maneuver_length", 15.0);
        log.debug("Load param minimumManeuverLength = " + minimumManeuverLength);
        timeHeadway = pluginServiceLocator.getParameterSource().getDouble("~platooning_desired_time_headway", 1.8);
        log.debug("Load param timeHeadway = " + timeHeadway);
        standStillGap = pluginServiceLocator.getParameterSource().getDouble("~platooning_min_gap", 7.0);
        log.debug("Load param standStillGap = " + standStillGap);
        kpPID = pluginServiceLocator.getParameterSource().getDouble("~platooning_Kp", 1.5);
        kiPID = pluginServiceLocator.getParameterSource().getDouble("~platooning_Ki", 0.0);
        kdPID = pluginServiceLocator.getParameterSource().getDouble("~platooning_Kd", 0.1);
        log.info("Parameters for speed PID controller are set to be [p = " + kpPID + ", i = " + kiPID + ", d = " + kdPID + "]");
        messageIntervalLength = pluginServiceLocator.getParameterSource().getInteger("~platooning_status_interval", 500);
        log.debug("Load param messageIntervalLength = " + messageIntervalLength);
        messageTimeout = (int) (messageIntervalLength * pluginServiceLocator.getParameterSource().getDouble("~platooning_status_timeout_factor", 1.5));
        log.debug("Load param messageTimeout = " + messageTimeout);
        // initialize necessary pubs/subs 
        mobilityIntroPublisher = pubSubService.getPublisherForTopic("mobility_intro_outbound", MobilityIntro._TYPE);
        newPlanSub = pubSubService.getSubscriberForTopic("new_plan", NewPlan._TYPE);
        cmdSpeedSub = pubSubService.getSubscriberForTopic(SPEED_CMD_CAPABILITY, SpeedAccel._TYPE);
        maneuverInputs = pluginServiceLocator.getManeuverPlanner().getManeuverInputs();
        // the plugin will start with standby state
        log.info("CACC platooning pulgin is initialized");
    }

    @Override
    public void onResume() {
        // reset plugin state to standby
        this.state = new StandbyState(this, log, pluginServiceLocator);
        log.info("The current CACC plugin state is " + this.state.toString());
        // Start all sub-threads
        if(platoonManager == null && platoonManagerThread == null) {
            platoonManager = new PlatoonManager(this, log);
            platoonManagerThread = new Thread(platoonManager);
            platoonManagerThread.setName("Platooning List Manager");
            platoonManagerThread.start();
            log.debug("Started platoonManagerThread.");
        }
        if(commandGenerator == null && commandGeneratorThread == null) {
            commandGenerator = new CommandGenerator(this, log, pluginServiceLocator);
            commandGeneratorThread = new Thread(commandGenerator);
            commandGeneratorThread.setName("Platooning Command Generator");
            commandGeneratorThread.start();
            log.debug("Started commandGeneratorThread.");
        }
        // register message callback on NewPlan message
        newPlanSub.registerOnMessageCallback((plan) -> state.onReceiveNegotiationMessage(plan));
        this.setAvailability(true);
    }

    @Override
    public void loop() throws InterruptedException {
        try {
            long loopStart = System.currentTimeMillis();
            MobilityIntro mobilityIntro = state.getNewOutboundIntroMessage(); 
            // Publish mobility Intro messages at a fixed rate if the current state of plugin wants to publish something
            if(mobilityIntro != null) {
                mobilityIntroPublisher.publish(state.getNewOutboundIntroMessage());
                log.debug("V2V", "Sending MobilityIntro to message node at state " + this.state.toString());
                log.debug("V2V", "    Plan ID = " + mobilityIntro.getHeader().getPlanId());
                log.debug("V2V", "    PlayType = " + mobilityIntro.getPlanType().getType());
                log.debug("V2V", "    My lane ID = " + mobilityIntro.getMyLaneId());
                log.debug("V2V", "    Forward speed = " + mobilityIntro.getForwardSpeed());
                log.debug("V2V", "    Capabilities = " + mobilityIntro.getCapabilities()); 
            }
            state.checkCurrentState();
            long loopStop = System.currentTimeMillis();
            long sleepTime = Math.max(0, messageIntervalLength - (loopStop - loopStart));
            Thread.sleep(sleepTime);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    @Override
    public void onSuspend() {
        this.setAvailability(false);
        if(commandGeneratorThread != null) {
            commandGeneratorThread.interrupt();
            commandGenerator = null;
            commandGeneratorThread = null;
        }
        if(platoonManagerThread != null) {
            platoonManagerThread.interrupt();
            platoonManager = null;
            platoonManagerThread = null;
        }
        // Reset to standby state as its default state
        this.state = new StandbyState(this, log, pluginServiceLocator);
    }

    @Override
    public void onTerminate() {
        // NO-OP
    }

    protected void setState(IPlatooningState state) {
        log.info("Platooning plugin is changing from " + this.state.toString() + " state to " + state.toString() + " state");
        this.state = state;
    }
    
    @Override
    public TrajectoryPlanningResponse planTrajectory(Trajectory traj, double expectedEntrySpeed) {
        log.info("Plan Trajectory from " + traj.toString() + " in state " + state.toString());
        return this.state.planTrajectory(traj, expectedEntrySpeed);
    }

}
