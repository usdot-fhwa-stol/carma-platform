/*
 * Copyright (C) 2017 LEIDOS.
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

package gov.dot.fhwa.saxton.carma.guidance;

import cav_msgs.*;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.IPubSubService;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.IPublisher;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.ISubscriber;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.OnMessageCallback;
import org.ros.node.ConnectedNode;

import java.util.ArrayList;
import java.util.List;

/**
 * Guidance sub-component responsible for the execution of maneuvers and generation of maneuver plans
 */
public class Maneuvers extends GuidanceComponent implements IStateChangeListener {
    
    private IPublisher<NewPlan> newPlanPublisher;
    private ISubscriber<RoadwayEnvironment> roadwayEnvironmentSubscriber;
    private ISubscriber<Route> routeSubscriber;
    
    Maneuvers(GuidanceStateMachine stateMachine, IPubSubService iPubSubService, ConnectedNode node) {
        super(stateMachine, iPubSubService, node);
        jobQueue.add(new Startup());
        stateMachine.registerStateChangeListener(this);
    }

    @Override public String getComponentName() {
        return "Guidance.Maneuvers";
    }
    
    protected class Startup implements Runnable {
        
        @Override
        public void run() {
            // Set up subscribers
            roadwayEnvironmentSubscriber = pubSubService.getSubscriberForTopic("roadway_environment",
                    RoadwayEnvironment._TYPE);

            roadwayEnvironmentSubscriber.registerOnMessageCallback(new OnMessageCallback<RoadwayEnvironment>() {
                @Override
                public void onMessage(RoadwayEnvironment msg) {
                    log.info("Received RoadwayEnvironment:" + msg);
                }
            });

            routeSubscriber = pubSubService.getSubscriberForTopic("route", Route._TYPE);

            routeSubscriber.registerOnMessageCallback(new OnMessageCallback<Route>() {
                @Override
                public void onMessage(Route msg) {
                    log.info("Received Route:" + msg);
                }
            });

            // Set up publishers
            newPlanPublisher = pubSubService.getPublisherForTopic("new_plan", NewPlan._TYPE);
            
            currentState.set(GuidanceState.STARTUP);
        }

    }

    protected class SystemReady implements Runnable {
        @Override
        public void run() {
            currentState.set(GuidanceState.DRIVERS_READY);
        }
    }

    protected class RouteActive implements Runnable {
        @Override
        public void run() {
            currentState.set(GuidanceState.ACTIVE);
        }
    }
    
    protected class Engage implements Runnable {
        @Override
        public void run() {
            currentState.set(GuidanceState.ENGAGED);
        }
    }
    
    protected class CleanRestart implements Runnable {
        @Override
        public void run() {
            currentState.set(GuidanceState.DRIVERS_READY);
        }
    }

    @Override
    public void onStateChange() {
        GuidanceState oldState = currentState.get();
        GuidanceState newState = stateMachine.getState();
        log.debug("GUIDANCE_STATE", getComponentName() + " changed state from " + oldState + " to " + newState);
        switch (oldState) {
        case STARTUP:
            if(newState == GuidanceState.SHUTDOWN) {
                jobQueue.add(new Shutdown(getComponentName() + " found GuidanceState changed to SHUTDOWN!"));
            } else if(newState == GuidanceState.DRIVERS_READY) {
                jobQueue.add(new SystemReady());
            }
            break;
        case DRIVERS_READY:
            if(newState == GuidanceState.SHUTDOWN) {
                jobQueue.add(new Shutdown(getComponentName() + " found GuidanceState changed to SHUTDOWN!"));
            } else if(newState == GuidanceState.ACTIVE) {
                jobQueue.add(new RouteActive());
            }
            break;
        case ACTIVE:
            if(newState == GuidanceState.SHUTDOWN) {
                jobQueue.add(new Shutdown(getComponentName() + " found GuidanceState changed to SHUTDOWN!"));
            } else if(newState == GuidanceState.ENGAGED) {
                jobQueue.add(new Engage());
            } else if(newState == GuidanceState.DRIVERS_READY) {
                jobQueue.add(new CleanRestart());
            }
            break;
        case ENGAGED:
            if(newState == GuidanceState.SHUTDOWN) {
                jobQueue.add(new Shutdown(getComponentName() + " found GuidanceState changed to SHUTDOWN!"));
            } else if(newState == GuidanceState.DRIVERS_READY) {
                jobQueue.add(new CleanRestart());
            }
            break;
        default:
            break;
        }
    }
    
    @Override public void timingLoop() {
        if (currentState.get() == GuidanceState.ENGAGED) {
            NewPlan newPlan = newPlanPublisher.newMessage();
            newPlan.setExpiration(node.getCurrentTime());
            newPlan.getHeader().setChecksum((short) 0);
            newPlan.getHeader().setPlanId(curPlanId++);
            newPlan.getHeader().setSenderId("test-cav1");
            newPlan.getHeader().setRecipientId("test-cav2");
            newPlan.getPlanType().setType(PlanType.JOIN_PLATOON_REAR);

            List<String> participantIds = new ArrayList<>();
            participantIds.add("test-cav1");
            participantIds.add("test-cav2");
            newPlan.setParticipantIds(participantIds);

            List<cav_msgs.Maneuver> maneuvers = new ArrayList<>();
            Maneuver m = node.getTopicMessageFactory().newFromType(Maneuver._TYPE);
            m.setLength(10);
            m.setPerformers(2);
            m.setStartRoadwayLaneId((byte) 0);
            m.setStartRoadwayLink("link1");
            m.setStartRoadwayOriginatorPosition(0);
            m.setType((byte) 8);
            maneuvers.add(m);
            newPlan.setManeuvers(maneuvers);

            newPlanPublisher.publish(newPlan);
        }

        try {
            Thread.sleep(sleepDurationMillis);
        } catch (InterruptedException e) {
            e.printStackTrace();
            Thread.currentThread().interrupt(); // Rethrow the interrupt
        }
    }

    protected long sleepDurationMillis = 30000;
    protected short curPlanId = 0;
}
