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

package gov.dot.fhwa.saxton.carma.guidance;

import gov.dot.fhwa.saxton.carma.guidance.pubsub.IPubSubService;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.IPublisher;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.ISubscriber;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.OnMessageCallback;

import org.ros.exception.RosRuntimeException;
import org.ros.exception.ServiceException;
import org.ros.node.ConnectedNode;
import org.ros.node.service.ServiceResponseBuilder;
import org.ros.node.service.ServiceServer;

import cav_msgs.RobotEnabled;
import cav_msgs.RouteEvent;
import cav_msgs.SystemAlert;
import cav_srvs.SetGuidanceActive;
import cav_srvs.SetGuidanceActiveRequest;
import cav_srvs.SetGuidanceActiveResponse;

/**
 * Handles final cleanup after Guidance shutdown
 * <p>
 * Also listens for route_state changes that would indicate the need for a Guidance shutdown
 */
public class GuidanceStateHandler extends GuidanceComponent implements IStateChangeListener {
    
    protected long shutdownDelayMs = 5000;
    protected boolean robotStatus = false;
    
    protected ISubscriber<RouteEvent> routeEventSub;
    protected ISubscriber<SystemAlert> systemAlertSub;
    protected ISubscriber<RobotEnabled> robotStatusSub;
    protected IPublisher<SystemAlert> systemAlertPub;
    protected ServiceServer<SetGuidanceActiveRequest, SetGuidanceActiveResponse> guidanceActiveService;

    public GuidanceStateHandler(GuidanceStateMachine stateMachine, IPubSubService pubSubService, ConnectedNode node) {
        super(stateMachine, pubSubService, node);
        stateMachine.registerStateChangeListener(this);
        jobQueue.add(this::onStartup);
    }

    @Override
    public String getComponentName() {
        return "GuidanceStateHandler";
    }

    @Override
    public void onStartup() {
        shutdownDelayMs = node.getParameterTree().getInteger("~shutdown_wait_time", 5000);
        
        systemAlertSub = pubSubService.getSubscriberForTopic("system_alert", SystemAlert._TYPE);
        systemAlertSub.registerOnMessageCallback(new OnMessageCallback<SystemAlert>() {
            
            @Override
            public void onMessage(SystemAlert msg) {
                if(msg.getType() == SystemAlert.DRIVERS_READY) {
                    log.info("GUIDANCE_STATE", getComponentName() + " received DRIVERS_READY");
                    stateMachine.processEvent(GuidanceEvent.FOUND_DRIVERS);
                } else if(msg.getType() == SystemAlert.FATAL) {
                    log.fatal("GUIDANCE_STATE", getComponentName() + "received FATAL");
                    stateMachine.processEvent(GuidanceEvent.PANIC);
                } else if(msg.getType() == SystemAlert.SHUTDOWN) {
                    log.warn("GUIDANCE_STATE", getComponentName() + "received SHUTDOWN");
                    stateMachine.processEvent(GuidanceEvent.SHUTDOWN);
                }
                
            }
        });
        systemAlertPub = pubSubService.getPublisherForTopic("system_alert", SystemAlert._TYPE);
        
        routeEventSub = pubSubService.getSubscriberForTopic("route_event", RouteEvent._TYPE);
        routeEventSub.registerOnMessageCallback((msg) -> {
            if (msg.getEvent() == RouteEvent.LEFT_ROUTE) {
                log.info("GUIDANCE_STATE", getComponentName() + " recieved LEFT_ROUTE");
                SystemAlert alert = systemAlertPub.newMessage();
                alert.setDescription("Guidance detected LEFT_ROUTE state!");
                alert.setType(SystemAlert.CAUTION);
                systemAlertPub.publish(alert);
                stateMachine.processEvent(GuidanceEvent.LEFT_ROUTE);
            } else if (msg.getEvent() == RouteEvent.ROUTE_COMPLETED || msg.getEvent() == RouteEvent.ROUTE_ABORTED) {
                log.info("GUIDANCE_STATE", getComponentName() + " recieved ROUTE_COMPLETED or ROUTE_ABORTED");
                SystemAlert alert = systemAlertPub.newMessage();
                alert.setDescription("Guidance detected ROUTE_COMPLETED or ROUTE_ABORTED state.");
                alert.setType(SystemAlert.CAUTION);
                systemAlertPub.publish(alert);
                stateMachine.processEvent(GuidanceEvent.FINISH_ROUTE);
            }
        });

        guidanceActiveService = node.newServiceServer("set_guidance_active", SetGuidanceActive._TYPE,
                new ServiceResponseBuilder<SetGuidanceActiveRequest, SetGuidanceActiveResponse>() {
                    @Override
                    public void build(SetGuidanceActiveRequest setGuidanceActiveRequest,
                            SetGuidanceActiveResponse setGuidanceActiveResponse) throws ServiceException {
                        if (setGuidanceActiveRequest.getGuidanceActive() && currentState.get() == GuidanceState.DRIVERS_READY) {
                            stateMachine.processEvent(GuidanceEvent.ACTIVATE);
                            setGuidanceActiveResponse.setGuidanceStatus(stateMachine.getState() == GuidanceState.ACTIVE);
                        } else if (!setGuidanceActiveRequest.getGuidanceActive()) {
                            // this will trigger deactivate or disengage, here we just use the same name - DISENGAGE
                            stateMachine.processEvent(GuidanceEvent.DISENGAGE);
                            setGuidanceActiveResponse.setGuidanceStatus(false);
                        } else {
                            setGuidanceActiveResponse.setGuidanceStatus(false);
                        }
                    }
                });

        currentState.set(GuidanceState.STARTUP);
    }

    @Override
    public void onSystemReady() {
        
        robotStatusSub = pubSubService.getSubscriberForTopic("robot_status", RobotEnabled._TYPE);
        if(robotStatusSub != null) {
            robotStatusSub.registerOnMessageCallback(new OnMessageCallback<RobotEnabled>() {
                @Override
                public void onMessage(RobotEnabled msg) {
                    if(!robotStatus && msg.getRobotActive()) {
                        stateMachine.processEvent(GuidanceEvent.START_ROUTE);
                        log.info("GUIDANCE_STATE", "Guidance StateMachine received START_ROUTE event");
                        robotStatus = true;
                    } else if(robotStatus && !msg.getRobotActive()) {
                        stateMachine.processEvent(GuidanceEvent.ROBOT_DISABLED);
                        log.info("GUIDANCE_STATE", "Guidance StateMachine received ROBOT_DISABLED event");
                        robotStatus = false;
                    }
                }
            });
        }
        currentState.set(GuidanceState.DRIVERS_READY);
    }

    @Override
    public void onActive() { 
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
    public void onShutdown() {
        currentState.set(GuidanceState.SHUTDOWN);
        log.info("SHUTDOWN",
                "Guidance state handler waiting" + shutdownDelayMs + "ms for guidance thread shutdown...");
        log.info("Publishing system_alert shutdown to ensure whole system shutdown");
        SystemAlert alert = systemAlertPub.newMessage();
        alert.setDescription("GuidanceStateHandler shutting down Guidance and CARMA platform");
        alert.setType(SystemAlert.SHUTDOWN);
        systemAlertPub.publish(alert);

        timingLoopThread.interrupt();
        try {
            Thread.sleep(shutdownDelayMs);
        } catch (InterruptedException e) {
            log.error("SHUTDOWN", "Guidance shutdown handler interrupted while waiting, cleanup may not have finished!", e);
        }
        log.info("SHUTDOWN", "Guidance state handler killing Guidance node.");
        loopThread.interrupt();
        node.shutdown();
    }

    @Override
    public void onPanic() {
        currentState.set(GuidanceState.SHUTDOWN);
        log.info("SHUTDOWN",
                "GuidanceStateHandler detected Guidance PANIC");
        log.fatal("Publishing FATAL system_alert shutdown to ensure whole system shutdown");
        SystemAlert alert = systemAlertPub.newMessage();
        alert.setDescription("GuidanceStateHandler detected Guidance PANIC! See log files for source.");
        alert.setType(SystemAlert.FATAL);
        systemAlertPub.publish(alert);

        timingLoopThread.interrupt();

        log.fatal("SHUTDOWN", "Guidance state handler killing Guidance node.");
        loopThread.interrupt();
        node.shutdown();
    }

    /*
     * This method add the right job in the jobQueue base on the instruction given by GuidanceStateMachine
     * The actual changing of GuidanceState local copy is happened when each job is performed
     */
    @Override
    public void onStateChange(GuidanceAction action) {
        log.debug("GUIDANCE_STATE", getComponentName() + " received action: " + action);
        switch (action) {
        case INTIALIZE:
            jobQueue.add(this::onSystemReady);
            break;
        case ACTIVATE:
            jobQueue.add(this::onActive);
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
        case RESTART:
            jobQueue.add(this::onCleanRestart);
            break;
        case PANIC_SHUTDOWN:
            jobQueue.add(this::onPanic);
            break;
        default:
            throw new RosRuntimeException(getComponentName() + " received unknown instruction from guidance state machine.");
        }
    }
}
