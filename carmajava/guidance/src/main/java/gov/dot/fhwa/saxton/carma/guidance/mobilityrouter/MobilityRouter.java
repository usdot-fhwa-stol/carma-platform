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

package gov.dot.fhwa.saxton.carma.guidance.mobilityrouter;

import java.util.Collections;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;

import org.ros.exception.RosRuntimeException;
import org.ros.node.ConnectedNode;
import cav_msgs.MobilityHeader;
import cav_msgs.MobilityOperation;
import cav_msgs.MobilityPath;
import cav_msgs.MobilityRequest;
import cav_msgs.MobilityResponse;
import gov.dot.fhwa.saxton.carma.guidance.GuidanceAction;
import gov.dot.fhwa.saxton.carma.guidance.GuidanceComponent;
import gov.dot.fhwa.saxton.carma.guidance.GuidanceState;
import gov.dot.fhwa.saxton.carma.guidance.GuidanceStateMachine;
import gov.dot.fhwa.saxton.carma.guidance.IStateChangeListener;
import gov.dot.fhwa.saxton.carma.guidance.conflictdetector.ConflictSpace;
import gov.dot.fhwa.saxton.carma.guidance.conflictdetector.IConflictManager;
import gov.dot.fhwa.saxton.carma.guidance.plugins.IPlugin;
import gov.dot.fhwa.saxton.carma.guidance.plugins.PluginManager;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.*;
import gov.dot.fhwa.saxton.carma.guidance.trajectory.TrajectoryExecutor;
import gov.dot.fhwa.saxton.carma.guidance.util.trajectoryconverter.ITrajectoryConverter;
import gov.dot.fhwa.saxton.carma.guidance.util.trajectoryconverter.RoutePointStamped;

/**
 * Mobility Message Routing Component for Guidance
 * <p>
 * Handles incoming MobilityResponse, MobilityPath, MobilityRequest, and MobilityOperations messages and 
 * route them to the appropriately registered handler callback based on the incoming messages strategy
 * string. Also handles ignoring messages not directed at the host vehicle.
 * <p>
 * Also handles publication of ACK/NACK responses to inbound MobilityRequest messages based on plugin
 * handler return codes.
 */
public class MobilityRouter extends GuidanceComponent implements IMobilityRouter, IStateChangeListener {

    private final String componentName = "MobilityRouter";
    //private IPublisher<MobilityRequest> bsmPublisher;
    private ISubscriber<MobilityRequest> requestSub;
    private ISubscriber<MobilityResponse> ackSub;
    private ISubscriber<MobilityOperation> operationSub;
    private ISubscriber<MobilityPath> pathSub;
    private IPublisher<MobilityResponse> ackPub;
    private Map<String, LinkedList<MobilityRequestHandler>> requestMap = Collections.synchronizedMap(new HashMap<>());
    private List<MobilityResponseHandler> ackList = Collections.synchronizedList(new LinkedList<>());
    private Map<String, LinkedList<MobilityOperationHandler>> operationMap = Collections.synchronizedMap(new HashMap<>());
    private Map<String, LinkedList<MobilityPathHandler>> pathMap = Collections.synchronizedMap(new HashMap<>());

    private PluginManager pluginManager;
    private TrajectoryExecutor trajectoryExecutor;
    private String defaultConflictHandlerName = "";
    private IPlugin defaultConflictHandler;
    private IConflictManager conflictManager;
    private ITrajectoryConverter trajectoryConverter;
    private String hostMobilityStaticId = "";

    public MobilityRouter(GuidanceStateMachine stateMachine, IPubSubService pubSubService, ConnectedNode node,
     IConflictManager conflictManager, ITrajectoryConverter trajectoryConverter,
     TrajectoryExecutor trajectoryExecutor) {
        super(stateMachine, pubSubService, node);
        this.conflictManager = conflictManager;
        this.trajectoryConverter = trajectoryConverter;
        this.trajectoryExecutor = trajectoryExecutor;

        stateMachine.registerStateChangeListener(this);
    }

    public void setPluginManager(PluginManager pluginManager) {
        this.pluginManager = pluginManager;
    }

    @Override
    public String getComponentName() {
        return componentName;
    }

    @Override
    public String getHostMobilityId() {
        return hostMobilityStaticId;
    }

    @Override
    public void onStartup() {
        log.info("Setting up subscribers");
        requestSub = pubSubService.getSubscriberForTopic("incoming_mobility_request", MobilityRequest._TYPE);
        ackSub = pubSubService.getSubscriberForTopic("incoming_mobility_response", MobilityResponse._TYPE);
        operationSub = pubSubService.getSubscriberForTopic("incoming_mobility_operation", MobilityOperation._TYPE);
        pathSub = pubSubService.getSubscriberForTopic("incoming_mobility_path", MobilityPath._TYPE);
        ackPub = pubSubService.getPublisherForTopic("outgoing_mobility_response", MobilityResponse._TYPE);

        requestSub.registerOnMessageCallback(this::handleMobilityRequest);
        ackSub.registerOnMessageCallback(this::handleMobilityResponse);
        operationSub.registerOnMessageCallback(this::handleMobilityOperation);
        pathSub.registerOnMessageCallback(this::handleMobilityPath);

        defaultConflictHandlerName = node.getParameterTree().getString("~default_mobility_conflict_handler", "Yield Plugin");
        hostMobilityStaticId = node.getParameterTree().getString("~vehicle_id", "");

        log.info("Setup complete");
    }

    @Override
    public void onSystemReady() {
        for (IPlugin p : pluginManager.getRegisteredPlugins()) {
            // Current constraint is that the default plugin must itself implement these interfaces
            if (p.getVersionInfo().componentName().equals(defaultConflictHandlerName)
            && p instanceof MobilityRequestHandler 
            && p instanceof MobilityPathHandler) {
                defaultConflictHandler = p;
                log.info("Detected default conflict handler: " + p.getVersionInfo());
            }
        }

        if (defaultConflictHandler == null) {
            log.warn("No default conflict handling strategy available! Platform will fail on first default conflict!!!");
        }
    }

    @Override
    public void onActive() {
        // NO-OP
    }

    @Override
    public void onEngaged() {
        // NO-OP
    }

    @Override
    public void onCleanRestart() {
        // NO-OP
    }

    @Override
    public void onDeactivate() {
        // NO-OP
    }
    
    private boolean isBroadcast(MobilityHeader header) {
        return header.getRecipientId().equals("");
    }

    /**
     * Handles the mobility request callback execution in a separate thread.
     * <p>
     * Also uses the plugin's returned {@link MobilityRequestResponse} value to determine how to proceed,
     * if nacked or ignored in special cases the path will not be added to the set of known paths in potential
     * conflict analysis.
     * 
     * @param handler the callback to be invoked in the background thread
     * @param msg The MobilityRequest message being handled
     * @param hasConflict True if the MobilityRequest message has a conflict that needs to be resolved, false o.w.
     * @param conflictSpace The spatial data describing the conflict
     */
    private void fireMobilityRequestCallback(MobilityRequestHandler handler, MobilityRequest msg, boolean hasConflict, ConflictSpace conflictSpace) {
        new Thread(() -> {
            MobilityRequestResponse resp = handler.handleMobilityRequestMessage(msg, hasConflict, conflictSpace);

            // Initialize the response message
            MobilityResponse respMsg = ackPub.newMessage();
            respMsg.getHeader().setPlanId(msg.getHeader().getPlanId());
            respMsg.getHeader().setRecipientId(msg.getHeader().getSenderId());
            respMsg.getHeader().setSenderId(hostMobilityStaticId);
            //respMsg.getHeader().setSenderBsmId(...); We don't have this data here, should this field be set in Message node?
            respMsg.getHeader().setTimestamp(System.currentTimeMillis());

            if (resp == MobilityRequestResponse.ACK) {
                List<RoutePointStamped> path = trajectoryConverter.messageToPath(msg.getTrajectory());
                conflictManager.addRequestedPath(path, msg.getHeader().getPlanId(), msg.getHeader().getSenderId());
                respMsg.setIsAccepted(true);
                ackPub.publish(respMsg);
            } else if (isBroadcast(msg.getHeader()) && resp == MobilityRequestResponse.NO_RESPONSE) {
                List<RoutePointStamped> path = trajectoryConverter.messageToPath(msg.getTrajectory());
                conflictManager.addRequestedPath(path, msg.getHeader().getPlanId(), msg.getHeader().getSenderId());
            } else if (resp == MobilityRequestResponse.NACK) {
                respMsg.setIsAccepted(false);
                ackPub.publish(respMsg);
            } // else don't send a response
        },
        "MobilityRequestHandlerCallback:" + handler.getClass().getSimpleName()).start();
    }

    /**
     * Handles the mobility response callback execution in a separate thread.
     * 
     * @param handler the callback to be invoked in the background thread
     * @param msg The MobilityResponse message being handled
     */
    private void fireMobilityResponseCallback(MobilityResponseHandler handler, MobilityResponse msg) {
        new Thread(() -> handler.handleMobilityResponseMessage(msg),
                "MobilityResponseHandlerCallback:" + handler.getClass().getSimpleName()).start();
    }

    /**
     * Handles the mobility operations callback execution in a separate thread.
     * 
     * @param handler the callback to be invoked in the background thread
     * @param msg The MobilityOperation message being handled
     */
    private void fireMobilityOperationCallback(MobilityOperationHandler handler, MobilityOperation msg) {
        new Thread(() -> handler.handleMobilityOperationMessage(msg),
                "MobilityOperationHandlerCallback:" + handler.getClass().getSimpleName()).start();
    }

    /**
     * Handles the mobility path callback execution in a separate thread.
     * 
     * @param handler the callback to be invoked in the background thread
     * @param msg The MobilityOperation message being handled
     * @param hasConflict True if the MobilityPath message has a conflict that needs to be resolved, false o.w.
     * @param conflictSpace The spatial data describing the conflict, null if has conflict is false
     */
    private void fireMobilityPathCallback(MobilityPathHandler handler, MobilityPath msg, boolean hasConflict,
			ConflictSpace conflictSpace) {
        new Thread(() -> {
            handler.handleMobilityPathMessageWithConflict(msg, hasConflict, conflictSpace);
        },
        "MobilityRequestHandlerCallback:" + handler.getClass().getSimpleName()).start();
	}


    /**
     * ROS messaging callback to be invoked upon receipt of an inbound MobilityRequest message
     * <p>
     * This method is responsible for ensuring that the inbound message is relevant to the host vehicle and analyzing
     * the path contained in the inbound message for any potential conflicts with the host vehicle's predicted path
     * prior to notifying any of the the registered callbacks of it's existence. If the message is not deemed relevant
     * or does not contain a conflict then no call is made.
     */
    private void handleMobilityRequest(MobilityRequest msg) {
        if (stateMachine.getState() != GuidanceState.ENGAGED) {
            return;
        }

        log.info("Handling incoming mobility request message: " + msg.getHeader().getPlanId() + " with strategy " + msg.getStrategy());

        if (!msg.getHeader().getRecipientId().equals(hostMobilityStaticId) && !isBroadcast(msg.getHeader())) {
            log.info("Message not destined for us, ignoring...");
            return;
        }
        List<RoutePointStamped> otherPath = trajectoryConverter.messageToPath(msg.getTrajectory());
        List<RoutePointStamped> hostPath = trajectoryExecutor.getHostPathPrediction();
        List<ConflictSpace> conflictSpaces = conflictManager.getConflicts(hostPath, otherPath);
        boolean conflictHandled = true;
        ConflictSpace conflictSpace = null;

        if (!conflictSpaces.isEmpty()) {
            conflictSpace = conflictSpaces.get(0); // Only use the first because the new trajectory will modify and change the others
            log.info(String.format("Conflict detected in path %s, startDist = %.02f, endDist = %.02f, lane = %d, startTime = %.02f, endTime = %.02f",
            msg.getHeader().getPlanId(),
            conflictSpace.getStartDowntrack(),
            conflictSpace.getEndDowntrack(),
            conflictSpace.getLane(),
            conflictSpace.getStartTime(),
            conflictSpace.getEndTime()));
            conflictHandled = false;
        }

        for (Entry<String, LinkedList<MobilityRequestHandler>> entry : requestMap.entrySet()) {
            if (entry.getKey().endsWith(msg.getStrategy())) {
                log.info("Firing message handlers registered for " + entry.getKey());
                for (MobilityRequestHandler handler : entry.getValue()) {
                    log.info("Firing mobility request handler: " + handler.getClass().getSimpleName());
                    fireMobilityRequestCallback(handler, msg, conflictSpace != null, conflictSpace);
                    conflictHandled = true;
                }
            }
        }

        if (conflictSpace != null) {
            if (!conflictHandled && defaultConflictHandler != null) {
                // Handle in default conflict handler
                log.info("No pre-registered handlers for the conflict were detected, defaulting to: " + defaultConflictHandler.getVersionInfo());
                fireMobilityRequestCallback(((MobilityRequestHandler) defaultConflictHandler), msg, true, conflictSpace);
            } else {
                throw new RosRuntimeException("Unhandled mobility path conflict detected and no default conflict handler available!!!");
            }
        }
    }

    /**
     * ROS message callback responsible for handling an inbound MobilityResponse message
     */
    private void handleMobilityResponse(MobilityResponse msg) {
        if (stateMachine.getState() != GuidanceState.ENGAGED) {
            return;
        }

        log.info("Processing incoming mobility ack message: " + msg.getHeader().getPlanId());

        if (!msg.getHeader().getRecipientId().equals(hostMobilityStaticId) && !isBroadcast(msg.getHeader())){
            log.info("Message not destined for us, ignoring...");
            return;
        }

        for (MobilityResponseHandler handler : ackList) {
            log.info("Firing message handler for " + handler.getClass().getSimpleName() + "with planId = " + msg.getHeader().getPlanId());
            fireMobilityResponseCallback(handler, msg);
        }
    }

    /**
     * ROS message callback responsible for handling an inbound MobilityOperation message
     */
    private void handleMobilityOperation(MobilityOperation msg) {
        if (stateMachine.getState() != GuidanceState.ENGAGED) {
            return;
        }

        log.info("Processing incoming mobility operation message: " + msg.getHeader().getPlanId());

        if (!msg.getHeader().getRecipientId().equals(hostMobilityStaticId) && !isBroadcast(msg.getHeader())) {
            log.info("Message not destined for us, ignoring...");
            return;
        }

        for (Entry<String, LinkedList<MobilityOperationHandler>> entry : operationMap.entrySet()) {
            if (entry.getKey().endsWith(msg.getStrategy())) {
                for (MobilityOperationHandler handler : entry.getValue()) {
                    log.info("Firing message handlers registered for " + entry.getKey());
                    fireMobilityOperationCallback(handler, msg);
                }
            }
        }
    }

    /**
     * ROS messaging callback to be invoked upon receipt of an inbound MobilityPath message
     * <p>
     * This method is responsible for ensuring that the inbound message is relevant to the host vehicle and analyzing
     * the path contained in the inbound message for any potential conflicts with the host vehicle's predicted path
     * prior to notifying any of the the registered callbacks of it's existence. If the message is not deemed relevant
     * or does not contain a conflict then no call is made.
     */
    private void handleMobilityPath(MobilityPath msg) {
        if (stateMachine.getState() != GuidanceState.ENGAGED) {
            return;
        }

        log.info("Processing incoming mobility path message: " + msg.getHeader().getPlanId());

        if (!msg.getHeader().getRecipientId().equals(hostMobilityStaticId) && !isBroadcast(msg.getHeader())) {
            log.info("Message not destined for us, ignoring...");
            return;
        }

        List<RoutePointStamped> hostTrajectory = trajectoryExecutor.getHostPathPrediction();
        List<RoutePointStamped> otherTrajectory = trajectoryConverter.messageToPath(msg.getTrajectory());
        conflictManager.addMobilityPath(otherTrajectory, msg.getHeader().getSenderId());
        List<ConflictSpace> conflictSpaces = conflictManager.getConflicts(hostTrajectory, otherTrajectory);

        if (!conflictSpaces.isEmpty()) {
            ConflictSpace conflictSpace = conflictSpaces.get(0); // Only use the first because the new trajectory will modify and change the others
            log.info(String.format("Conflict detected in path %s, startDist = %.02f, endDist = %.02f, lane = %d, startTime = %.02f, endTime = %.02f",
            msg.getHeader().getPlanId(),
            conflictSpace.getStartDowntrack(),
            conflictSpace.getEndDowntrack(),
            conflictSpace.getLane(),
            conflictSpace.getStartTime(),
            conflictSpace.getEndTime()));
            // Handle in default conflict handler
            log.info("Handling path conflict with " + defaultConflictHandler.getVersionInfo());
            fireMobilityPathCallback(((MobilityPathHandler) defaultConflictHandler), msg, true, conflictSpace);
        }
    }

    @Override
	public void registerMobilityRequestHandler(String strategyId, MobilityRequestHandler handler) {
        log.info("Mobility Request handler: " + handler.getClass().getSimpleName() + " registered for " + strategyId);
        if (!requestMap.containsKey(strategyId)) {
            requestMap.put(strategyId, new LinkedList<MobilityRequestHandler>());
        }
        if(!requestMap.get(strategyId).contains(handler)) {
            requestMap.get(strategyId).add(handler);
        }
    }

    @Override
    public void registerMobilityResponseHandler(MobilityResponseHandler handler) {
        log.info("Mobility Response handler: " + handler.getClass().getSimpleName() + " registered");
        if(!ackList.contains(handler)) {
            ackList.add(handler);
        }
    }

    @Override
    public void registerMobilityOperationHandler(String strategyId, MobilityOperationHandler handler) {
        log.info("Mobility Operation handler: " + handler.getClass().getSimpleName() + " registered for " + strategyId);
        if (!operationMap.containsKey(strategyId)) {
            operationMap.put(strategyId, new LinkedList<MobilityOperationHandler>());
        }
        if(!operationMap.get(strategyId).contains(handler)) {
            operationMap.get(strategyId).add(handler);
        }
    }

    @Override
    public void registerMobilityPathHandler(String strategyId, MobilityPathHandler handler) {
        log.info("Mobility Path handler: " + handler.getClass().getSimpleName() + " registered for " + strategyId);
        if (!pathMap.containsKey(strategyId)) {
            pathMap.put(strategyId, new LinkedList<MobilityPathHandler>());
        }
        if(!pathMap.get(strategyId).contains(handler)) {
            pathMap.get(strategyId).add(handler);
        }
    }

  @Override
  public void onStateChange(GuidanceAction action) {
    log.debug("GUIDANCE_STATE", getComponentName() + " received action: " + action);
    switch (action) {
    case INTIALIZE:
      jobQueue.add(this::onStartup);
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

}
