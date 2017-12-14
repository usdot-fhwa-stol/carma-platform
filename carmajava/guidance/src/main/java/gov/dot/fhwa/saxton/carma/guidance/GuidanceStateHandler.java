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

import gov.dot.fhwa.saxton.carma.guidance.pubsub.IPubSubService;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.IPublisher;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.IService;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.ISubscriber;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.OnMessageCallback;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.OnServiceResponseCallback;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.TopicNotFoundException;

import java.util.ArrayList;
import java.util.List;

import org.ros.exception.RosRuntimeException;
import org.ros.exception.ServiceException;
import org.ros.node.ConnectedNode;
import org.ros.node.service.ServiceResponseBuilder;
import org.ros.node.service.ServiceServer;

import cav_msgs.RobotEnabled;
import cav_msgs.RouteState;
import cav_msgs.SystemAlert;
import cav_srvs.GetDriversWithCapabilities;
import cav_srvs.GetDriversWithCapabilitiesRequest;
import cav_srvs.GetDriversWithCapabilitiesResponse;
import cav_srvs.SetGuidanceEngaged;
import cav_srvs.SetGuidanceEngagedRequest;
import cav_srvs.SetGuidanceEngagedResponse;

/**
 * Handles final cleanup after Guidance shutdown
 * <p>
 * Also listens for route_state changes that would indicate the need for a Guidance shutdown
 */
public class GuidanceStateHandler extends GuidanceComponent implements IStateChangeListener {
    
    protected final String ROBOTIC_STATUS = "control/robot_status";
    
    protected long shutdownDelayMs = 5000;
    
    protected ISubscriber<RouteState> routeStateSub;
    protected ISubscriber<SystemAlert> systemAlertSub;
    protected ISubscriber<RobotEnabled> robotStatusSub;
    protected IPublisher<SystemAlert> systemAlertPub;
    protected IService<GetDriversWithCapabilitiesRequest, GetDriversWithCapabilitiesResponse> driverCapabilityService;
    
    protected ServiceServer<SetGuidanceEngagedRequest, SetGuidanceEngagedResponse> guidanceEngageService;

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
                } else if(msg.getType() == SystemAlert.FATAL || msg.getType() == SystemAlert.SHUTDOWN) {
                    log.fatal("GUIDANCE_STATE", getComponentName() + "received FATAL or SHUTDOWN");
                    stateMachine.processEvent(GuidanceEvent.PANIC);
                }
                
            }
        });
        systemAlertPub = pubSubService.getPublisherForTopic("system_alert", SystemAlert._TYPE);
        
        routeStateSub = pubSubService.getSubscriberForTopic("route_state", RouteState._TYPE);
        routeStateSub.registerOnMessageCallback(new OnMessageCallback<RouteState>() {

            @Override
            public void onMessage(RouteState msg) {
                if(msg.getEvent() == RouteState.LEFT_ROUTE) {
                    log.info("GUIDANCE_STATE", getComponentName() + " recieved LEFT_ROUTE");
                    SystemAlert alert = systemAlertPub.newMessage();
                    alert.setDescription("Guidance detected LEFT_ROUTE state!");
                    alert.setType(SystemAlert.CAUTION);
                    systemAlertPub.publish(alert);
                    stateMachine.processEvent(GuidanceEvent.LEFT_ROUTE);
                } else if (msg.getEvent() == RouteState.ROUTE_COMPLETED
                        || msg.getEvent() == RouteState.ROUTE_ABORTED) {
                    log.info("GUIDANCE_STATE", getComponentName() + " recieved ROUTE_COMPLETED or ROUTE_ABORTED");
                    SystemAlert alert = systemAlertPub.newMessage();
                    alert.setDescription("Guidance detected ROUTE_COMPLETED or ROUTE_ABORTED state.");
                    alert.setType(SystemAlert.CAUTION);
                    systemAlertPub.publish(alert);
                    stateMachine.processEvent(GuidanceEvent.FINISH_ROUTE);
                }
            }
        });

        guidanceEngageService = node.newServiceServer("set_guidance_engaged", SetGuidanceEngaged._TYPE,
                new ServiceResponseBuilder<SetGuidanceEngagedRequest, SetGuidanceEngagedResponse>() {
                    @Override
                    public void build(SetGuidanceEngagedRequest setGuidanceEngagedRequest,
                            SetGuidanceEngagedResponse setGuidanceEngagedResponse) throws ServiceException {
                        if (setGuidanceEngagedRequest.getGuidanceEngage() && currentState.get() == GuidanceState.DRIVERS_READY) {
                            stateMachine.processEvent(GuidanceEvent.ACTIVATE_ROUTE);
                            setGuidanceEngagedResponse.setGuidanceStatus(stateMachine.getState() == GuidanceState.INACTIVE);
                        } else if (!setGuidanceEngagedRequest.getGuidanceEngage()) {
                            stateMachine.processEvent(GuidanceEvent.DISENGAGE);
                            setGuidanceEngagedResponse.setGuidanceStatus(false);
                        } else {
                            setGuidanceEngagedResponse.setGuidanceStatus(false);
                        }
                    }
                });

        currentState.set(GuidanceState.STARTUP);
    }

    @Override
    public void onSystemReady() {
        try {
            driverCapabilityService = pubSubService.getServiceForTopic("get_drivers_with_capabilities",
                    GetDriversWithCapabilities._TYPE);
        } catch (TopicNotFoundException tnfe) {
            stateMachine.processEvent(GuidanceEvent.PANIC);
            log.fatal("SHUTDOWN", "Interface manager not found.");
        }
        
        // Build request message
        GetDriversWithCapabilitiesRequest req = driverCapabilityService.newMessage();

        List<String> reqdCapabilities = new ArrayList<>();
        reqdCapabilities.add(ROBOTIC_STATUS);
        req.setCapabilities(reqdCapabilities);

        // Work around to pass a final object into our anonymous inner class so we can get the response
        final GetDriversWithCapabilitiesResponse[] drivers = new GetDriversWithCapabilitiesResponse[1];
        drivers[0] = null;

        // Call the InterfaceManager to see if we have a driver that matches our requirements
        driverCapabilityService.callSync(req, new OnServiceResponseCallback<GetDriversWithCapabilitiesResponse>() {
            @Override
            public void onSuccess(GetDriversWithCapabilitiesResponse msg) {
                log.debug("Received GetDriversWithCapabilitiesResponse");
                for (String driverName : msg.getDriverData()) {
                    log.debug(getComponentName() + " discovered driver: " + driverName);
                }
                drivers[0] = msg;
            }

            @Override
            public void onFailure(Exception e) {
                stateMachine.processEvent(GuidanceEvent.PANIC);
                log.fatal("InterfaceManager failed to return a control/robot_status capable driver!!!");
            }
        });
        
        String robotStatusTopic = null;
        if(drivers[0] != null) {
            for(String url : drivers[0].getDriverData()) {
                if(url.endsWith(ROBOTIC_STATUS)) {
                    robotStatusTopic = url;
                    break;
                }
            }                
        }
        
        if(robotStatusTopic != null) {
            log.debug(getComponentName() + " is connecting to " + robotStatusTopic);
            robotStatusSub = pubSubService.getSubscriberForTopic(robotStatusTopic, RobotEnabled._TYPE);
            if(robotStatusSub != null) {
                robotStatusSub.registerOnMessageCallback(new OnMessageCallback<RobotEnabled>() {
                    @Override
                    public void onMessage(RobotEnabled msg) {
                        if(msg.getRobotActive()) {
                            stateMachine.processEvent(GuidanceEvent.START_ROUTE);
                            log.info("GUIDANCE_STATE", "Guidance StateMachine received START_ROUTE event");
                        }
                    }
                });
            }
        }
        
        currentState.set(GuidanceState.DRIVERS_READY);
    }

    @Override
    public void onRouteActive() { 
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
        node.shutdown();
        loopThread.interrupt();
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
            jobQueue.add(this::onRouteActive);
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
        default:
            throw new RosRuntimeException(getComponentName() + "received unknow instruction from guidance state machine.");
        }
    }
    
}
