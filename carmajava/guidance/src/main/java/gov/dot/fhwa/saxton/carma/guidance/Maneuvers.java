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

package gov.dot.fhwa.saxton.carma.guidance;

import cav_msgs.*;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.IPubSubService;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.ISubscriber;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.OnMessageCallback;

import org.ros.exception.RosRuntimeException;
import org.ros.node.ConnectedNode;
/**
 * Guidance sub-component responsible for the execution of maneuvers and generation of maneuver plans
 */
public class Maneuvers extends GuidanceComponent implements IStateChangeListener {
    
    private ISubscriber<RoadwayEnvironment> roadwayEnvironmentSubscriber;
    private ISubscriber<Route> routeSubscriber;
    protected long sleepDurationMillis = 30000;
    protected short curPlanId = 0;
    
    Maneuvers(GuidanceStateMachine stateMachine, IPubSubService iPubSubService, ConnectedNode node) {
        super(stateMachine, iPubSubService, node);
        jobQueue.add(this::onStartup);
        stateMachine.registerStateChangeListener(this);
    }

    @Override
    public String getComponentName() {
        return "Guidance.Maneuvers";
    }
    
    @Override
    public void onStartup() {
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
        
        currentState.set(GuidanceState.STARTUP);
    }

    @Override
    public void onSystemReady() {
        currentState.set(GuidanceState.DRIVERS_READY);
    }
    
    @Override
    public void onRouteActive() {
        currentState.set(GuidanceState.ACTIVE);
    }
    
    @Override
    public void onDeactivate() {
        currentState.set(GuidanceState.INACTIVE);
    }
    
    @Override
    public void onEngaged() {
        currentState.set(GuidanceState.ENGAGED);
    }
    
    @Override
    public void onCleanRestart() {
        currentState.set(GuidanceState.DRIVERS_READY);
    }

    @Override
    public void onStateChange(GuidanceAction action) {
        log.debug("GUIDANCE_STATE", getComponentName() + " received action: " + action);
        switch (action) {
        case INTIALIZE:
            jobQueue.add(this::onSystemReady);
            break;
        case ACTIVATE:
            jobQueue.add(this::onRouteActive);
            break;
        case DEACTIVATE:
            jobQueue.add(this::onDeactivate);
            break;
        case ENGAGE:
            jobQueue.add(this::onEngaged);
            break;
        case SHUTDOWN:
            jobQueue.add(this::onShutdown);
            break;
        case PANIC_SHUTDOWN:
            jobQueue.add(this::onPanic);
            break;
        case RESTART:
            jobQueue.add(this::onCleanRestart);
            break;
        default:
            throw new RosRuntimeException(getComponentName() + "received unknow instruction from guidance state machine.");
        }
    }
    
    @Override public void timingLoop() {
        if (currentState.get() == GuidanceState.ENGAGED) {
            
        }

        try {
            Thread.sleep(sleepDurationMillis);
        } catch (InterruptedException e) {
            e.printStackTrace();
            Thread.currentThread().interrupt(); // Rethrow the interrupt
        }
    }
}
