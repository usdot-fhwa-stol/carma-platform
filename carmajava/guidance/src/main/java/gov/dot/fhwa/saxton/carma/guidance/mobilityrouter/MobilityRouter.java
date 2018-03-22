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
import java.util.concurrent.atomic.AtomicReference;

import org.ros.exception.RosRuntimeException;
import org.ros.node.ConnectedNode;
import cav_msgs.MobilityAck;
import cav_msgs.MobilityAckType;
import cav_msgs.MobilityHeader;
import cav_msgs.MobilityOperation;
import cav_msgs.MobilityPath;
import cav_msgs.MobilityRequest;
import cav_msgs.Route;
import cav_msgs.RouteState;
import gov.dot.fhwa.saxton.carma.guidance.GuidanceComponent;
import gov.dot.fhwa.saxton.carma.guidance.GuidanceStateMachine;
import gov.dot.fhwa.saxton.carma.guidance.arbitrator.Arbitrator;
import gov.dot.fhwa.saxton.carma.guidance.conflictdetector.ConflictSpace;
import gov.dot.fhwa.saxton.carma.guidance.conflictdetector.IConflictManager;
import gov.dot.fhwa.saxton.carma.guidance.plugins.IPlugin;
import gov.dot.fhwa.saxton.carma.guidance.plugins.PluginManager;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.*;
import gov.dot.fhwa.saxton.carma.guidance.trajectory.Trajectory;
import gov.dot.fhwa.saxton.carma.guidance.trajectory.TrajectoryExecutor;
import gov.dot.fhwa.saxton.carma.guidance.util.ILogger;
import gov.dot.fhwa.saxton.carma.guidance.util.LoggerManager;
import gov.dot.fhwa.saxton.carma.guidance.util.trajectoryconverter.ITrajectoryConverter;
import gov.dot.fhwa.saxton.carma.guidance.util.trajectoryconverter.RoutePointStamped;

/**
 * Mobility Message Routing Component for Guidance
 * <p>
 * Handles incoming MobilityAck, MobilityPath, MobilityRequest, and MobilityOperations messages and 
 * route them to the appropriately registered handler callback based on the incoming messages strategy
 * string. Also handles ignoring messages not directed at the host vehicle.
 * <p>
 * Also handles publication of ACK/NACK responses to inbound MobilityRequest messages based on plugin
 * handler return codes.
 */
public class MobilityRouter extends GuidanceComponent implements IMobilityRouter {

    private final String componentName = "MobilityRouter";
    //private IPublisher<MobilityRequest> bsmPublisher;
    private ISubscriber<MobilityRequest> requestSub;
    private ISubscriber<MobilityAck> ackSub;
    private ISubscriber<MobilityOperation> operationSub;
    private ISubscriber<MobilityPath> pathSub;
    private ISubscriber<Route> routeSub;
    private ISubscriber<RouteState> routeStateSub;
    private IPublisher<MobilityAck> ackPub;
    private Map<String, LinkedList<MobilityRequestHandler>> requestMap = Collections.synchronizedMap(new HashMap<>());
    private Map<String, LinkedList<MobilityAckHandler>> ackMap = Collections.synchronizedMap(new HashMap<>());
    private Map<String, LinkedList<MobilityOperationHandler>> operationMap = Collections.synchronizedMap(new HashMap<>());
    private Map<String, LinkedList<MobilityPathHandler>> pathMap = Collections.synchronizedMap(new HashMap<>());
    private AtomicReference<Route> curRoute = new AtomicReference<>();
    private AtomicReference<RouteState> curRouteState = new AtomicReference<>();

    private PluginManager pluginManager;
    private TrajectoryExecutor trajectoryExecutor;
    private String defaultConflictHandlerName = "";
    private IPlugin defaultConflictHandler;
    private IConflictManager conflictManager;
    private ITrajectoryConverter trajectoryConverter;
    private String hostMobilityStaticId = "";

    public MobilityRouter(GuidanceStateMachine stateMachine, IPubSubService pubSubService, ConnectedNode node, IConflictManager conflictManager, ITrajectoryConverter trajectoryConverter, TrajectoryExecutor trajectoryExecutor) {
        super(stateMachine, pubSubService, node);
        this.conflictManager = conflictManager;
        this.trajectoryConverter = trajectoryConverter;
        this.trajectoryExecutor = trajectoryExecutor;
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
        ackSub = pubSubService.getSubscriberForTopic("incoming_mobility_ack", MobilityAck._TYPE);
        operationSub = pubSubService.getSubscriberForTopic("incoming_mobility_operation", MobilityOperation._TYPE);
        pathSub = pubSubService.getSubscriberForTopic("incoming_mobility_path", MobilityPath._TYPE);
        routeSub = pubSubService.getSubscriberForTopic("route", Route._TYPE);
        routeStateSub = pubSubService.getSubscriberForTopic("route_state", RouteState._TYPE);
        ackPub = pubSubService.getPublisherForTopic("outbound_mobility_ack", MobilityAck._TYPE);

        requestSub.registerOnMessageCallback(this::handleMobilityRequest);
        ackSub.registerOnMessageCallback(this::handleMobilityAck);
        operationSub.registerOnMessageCallback(this::handleMobilityOperation);
        pathSub.registerOnMessageCallback(this::handleMobilityPath);
        routeSub.registerOnMessageCallback((msg) -> curRoute.set(msg));
        routeStateSub.registerOnMessageCallback((msg) -> curRouteState.set(msg));

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
        return header.getSenderId().equals("");
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
            MobilityAck respMsg = ackPub.newMessage();
            respMsg.getHeader().setPlanId(msg.getHeader().getPlanId());
            respMsg.getHeader().setRecipientId(msg.getHeader().getSenderId());
            respMsg.getHeader().setSenderId(hostMobilityStaticId);
            //respMsg.getHeader().setSenderBsmId(...); We don't have this data here, shoud this field be set in Message node?
            respMsg.getHeader().setTimestamp(System.currentTimeMillis());

            if (resp == MobilityRequestResponse.ACK) {
                List<RoutePointStamped> path = trajectoryConverter.messageToPath(msg.getTrajectory(), curRoute.get(), curRouteState.get());
                conflictManager.addRequestedPath(path, msg.getHeader().getPlanId(), msg.getHeader().getSenderId());
                respMsg.getAgreement().setType(MobilityAckType.ACCEPT_WITH_EXECUTE);
                ackPub.publish(respMsg);
            } else if (isBroadcast(msg.getHeader()) && resp == MobilityRequestResponse.NO_RESPONSE) {
                List<RoutePointStamped> path = trajectoryConverter.messageToPath(msg.getTrajectory(), curRoute.get(), curRouteState.get());
                conflictManager.addRequestedPath(path, msg.getHeader().getPlanId(), msg.getHeader().getSenderId());
            } else if (resp == MobilityRequestResponse.NACK) {
                respMsg.getAgreement().setType(MobilityAckType.REJECT);
                ackPub.publish(respMsg);
            }
        },
        "MobilityRequestHandlerCallback:" + handler.getClass().getSimpleName()).start();
    }

    /**
     * Handles the mobility ack callback execution in a separate thread.
     * 
     * @param handler the callback to be invoked in the background thread
     * @param msg The MobilityAck message being handled
     */
    private void fireMobilityAckCallback(MobilityAckHandler handler, MobilityAck msg) {
        new Thread(() -> handler.handleMobilityAckMessage(msg),
                "MobilityAckHandlerCallback:" + handler.getClass().getSimpleName()).start();
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
        log.info("Handling incoming mobility request message: " + msg.getHeader().getPlanId() + " with strategy " + msg.getStrategy());

        if (curRoute.get() != null || curRouteState.get() != null) {
            log.warn("Message received prior to route being inited, ignoring...");
            return;
        }

        if (!msg.getHeader().getRecipientId().equals(hostMobilityStaticId) || !msg.getHeader().getRecipientId().isEmpty()) {
            log.info("Message not destined for us, ignoring...");
            return;
        }
        List<RoutePointStamped> otherPath = trajectoryConverter.messageToPath(msg.getTrajectory(), curRoute.get(), curRouteState.get());
        List<RoutePointStamped> hostPath = trajectoryExecutor.getHostPathPrediction();
        List<ConflictSpace> conflictSpaces = conflictManager.getConflicts(hostPath, otherPath);

        if (!conflictSpaces.isEmpty()) {
            ConflictSpace conflictSpace = conflictSpaces.get(0); // Only use the first because the new trajectory will modify and change the others
            log.info(String.format("Conflict detected in request %s, startDist = %.02f, endDist = %.02f, startTime = %.02f, endTime = %.02f",
            msg.getHeader().getPlanId(),
            conflictSpace.getStartDowntrack(),
            conflictSpace.getEndDowntrack(),
            conflictSpace.getStartTime(),
            conflictSpace.getEndTime()));

            boolean conflictHandled = false;
            for (Entry<String, LinkedList<MobilityRequestHandler>> entry : requestMap.entrySet()) {
                if (entry.getKey().endsWith(msg.getStrategy())) {
                    log.info("Firing message handlers registered for " + entry.getKey());
                    for (MobilityRequestHandler handler : entry.getValue()) {
                        log.info("Firing mobility request handler: " + handler.getClass().getSimpleName());
                        fireMobilityRequestCallback(handler, msg, true, conflictSpace);
                        conflictHandled = true;
                    }
                }
            }

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
     * ROS message callback responsible for handling an inbound MobilityAck message
     */
    private void handleMobilityAck(MobilityAck msg) {
        log.info("Processing incoming mobility ack message: " + msg.getHeader().getPlanId());

        if (curRoute.get() != null || curRouteState.get() != null) {
            log.warn("Message received prior to route being inited, ignoring...");
            return;
        }

        if (!msg.getHeader().getRecipientId().equals(hostMobilityStaticId) || !msg.getHeader().getRecipientId().isEmpty()) {
            log.info("Message not destined for us, ignoring...");
            return;
        }

        for (Entry<String, LinkedList<MobilityAckHandler>> entry : ackMap.entrySet()) {
            if (entry.getKey().endsWith(msg.getHeader().getPlanId())) {
                log.info("Firing message handlers registered for " + entry.getKey());
                for (MobilityAckHandler handler : entry.getValue()) {
                    log.info("Firing mobility ack handler: " + handler.getClass().getSimpleName());
                    fireMobilityAckCallback(handler, msg);
                }
            }
        }
    }

    /**
     * ROS message callback responsible for handling an inbound MobilityOperation message
     */
    private void handleMobilityOperation(MobilityOperation msg) {
        log.info("Processing incoming mobility operation message: " + msg.getHeader().getPlanId());

        if (curRoute.get() != null || curRouteState.get() != null) {
            log.warn("Message received prior to route being inited, ignoring...");
            return;
        }

        if (!msg.getHeader().getRecipientId().equals(hostMobilityStaticId) || !msg.getHeader().getRecipientId().isEmpty()) {
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
        log.info("Processing incoming mobility path message: " + msg.getHeader().getPlanId());

        if (curRoute.get() != null || curRouteState.get() != null) {
            log.warn("Message received prior to route being inited, ignoring...");
            return;
        }

        if (!msg.getHeader().getRecipientId().equals(hostMobilityStaticId) || !msg.getHeader().getRecipientId().isEmpty()) {
            log.info("Message not destined for us, ignoring...");
            return;
        }

        List<RoutePointStamped> hostTrajectory = trajectoryExecutor.getHostPathPrediction();
        List<RoutePointStamped> otherTrajectory = trajectoryConverter.messageToPath(msg.getTrajectory(), curRoute.get(), curRouteState.get());
        conflictManager.addMobilityPath(otherTrajectory, msg.getHeader().getSenderId());
        List<ConflictSpace> conflictSpaces = conflictManager.getConflicts(hostTrajectory, otherTrajectory);

        if (!conflictSpaces.isEmpty()) {
            ConflictSpace conflictSpace = conflictSpaces.get(0); // Only use the first because the new trajectory will modify and change the others
            log.info(String.format("Conflict detected in path %s, startDist = %.02f, endDist = %.02f, startTime = %.02f, endTime = %.02f",
            msg.getHeader().getPlanId(),
            conflictSpace.getStartDowntrack(),
            conflictSpace.getEndDowntrack(),
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
        requestMap.get(strategyId).add(handler);
    }

    @Override
    public void registerMobilityAckHandler(String strategyId, MobilityAckHandler handler) {
        log.info("Mobility Ack handler: " + handler.getClass().getSimpleName() + " registered for " + strategyId);
        if (!ackMap.containsKey(strategyId)) {
            ackMap.put(strategyId, new LinkedList<MobilityAckHandler>());
        }
        ackMap.get(strategyId).add(handler);
    }

    @Override
    public void registerMobilityOperationHandler(String strategyId, MobilityOperationHandler handler) {
        log.info("Mobility Operation handler: " + handler.getClass().getSimpleName() + " registered for " + strategyId);
        if (!operationMap.containsKey(strategyId)) {
            operationMap.put(strategyId, new LinkedList<MobilityOperationHandler>());
        }
        operationMap.get(strategyId).add(handler);
    }

    @Override
    public void registerMobilityPathHandler(String strategyId, MobilityPathHandler handler) {
        log.info("Mobility Path handler: " + handler.getClass().getSimpleName() + " registered for " + strategyId);
        if (!pathMap.containsKey(strategyId)) {
            pathMap.put(strategyId, new LinkedList<MobilityPathHandler>());
        }
        pathMap.get(strategyId).add(handler);
    }
}
