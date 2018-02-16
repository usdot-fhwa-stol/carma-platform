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
    protected final String SPEED_CMD_CAPABILITY = "/saxton_cav/drivers/srx_controller/control/cmd_speed";
    
    protected IPlatooningState state;
    protected IPublisher<MobilityIntro> mobilityIntroPublisher;
    protected ISubscriber<NewPlan> newPlanSub;
    protected ISubscriber<SpeedAccel> cmdSpeedSub;
    protected IManeuverInputs maneuverInputs;
    
    protected double maxAccel = 2.5;
    protected double minimumManeuverLength = 15.0;
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
        maxAccel = pluginServiceLocator.getParameterSource().getDouble("~platooning_max_accel", 2.5);
        minimumManeuverLength = pluginServiceLocator.getParameterSource().getDouble("~platooning_min_maneuver_length", 5.0);
        messageIntervalLength = pluginServiceLocator.getParameterSource().getInteger("~platooning_status_interval", 500);
        messageTimeout = (int) (messageIntervalLength * pluginServiceLocator.getParameterSource().getDouble("~platooning_status_timeout_factor", 1.5));
        mobilityIntroPublisher = pubSubService.getPublisherForTopic("mobility_intro_outbound", MobilityIntro._TYPE);
        newPlanSub = pubSubService.getSubscriberForTopic("new_plan", NewPlan._TYPE);
        cmdSpeedSub = pubSubService.getSubscriberForTopic(SPEED_CMD_CAPABILITY, SpeedAccel._TYPE);
        maneuverInputs = pluginServiceLocator.getManeuverPlanner().getManeuverInputs();
        this.state = new StandbyState(this, log, pluginServiceLocator);
        log.info("CACC platooning pulgin is initialized");
    }

    @Override
    public void onResume() {
        this.state = new StandbyState(this, log, pluginServiceLocator);
        if(commandGenerator == null && commandGeneratorThread == null) {
            commandGenerator = new CommandGenerator(this);
            commandGeneratorThread = new Thread(commandGenerator);
            commandGeneratorThread.setName("Platooning Command Generator");
            commandGeneratorThread.start();
        }
        if(platoonManager == null && platoonManagerThread == null) {
            platoonManager = new PlatoonManager(this, log);
            platoonManagerThread = new Thread(platoonManager);
            platoonManagerThread.setName("Platooning List Manager");
            platoonManagerThread.start();
        }
        newPlanSub.registerOnMessageCallback((plan) -> state.onReceiveNegotiationMessage(plan));
        this.setAvailability(true);
    }

    @Override
    public void loop() throws InterruptedException {
        try {
            long loopStart = System.currentTimeMillis();
            MobilityIntro mobilityIntro = state.getNewOutboundIntroMessage(); 
            if(mobilityIntro != null) {
                mobilityIntroPublisher.publish(state.getNewOutboundIntroMessage());
            }
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
        this.state = new StandbyState(this, log, pluginServiceLocator);
    }

    @Override
    public void onTerminate() {
        // NO-OP
    }

    protected void setState(IPlatooningState state) {
        log.info("Platooning plugin change from " + this.state.toString() + " state to " + state.toString() + " state");
        this.state = state;
    }
    
    @Override
    public TrajectoryPlanningResponse planTrajectory(Trajectory traj, double expectedEntrySpeed) {
        log.info("Plan Trajectory from " + traj.getStartLocation() + " to " + traj.getEndLocation() + " in state " + state.toString());
        return this.state.planTrajectory(traj, expectedEntrySpeed);
    }

}
