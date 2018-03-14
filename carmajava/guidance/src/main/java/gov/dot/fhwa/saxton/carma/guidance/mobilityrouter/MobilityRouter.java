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

import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.concurrent.ConcurrentHashMap;
import org.ros.exception.RosRuntimeException;
import org.ros.node.ConnectedNode;
import cav_msgs.MobilityAck;
import cav_msgs.MobilityHeader;
import cav_msgs.MobilityOperation;
import cav_msgs.MobilityPath;
import cav_msgs.MobilityRequest;
import gov.dot.fhwa.saxton.carma.guidance.GuidanceComponent;
import gov.dot.fhwa.saxton.carma.guidance.GuidanceExceptionHandler;
import gov.dot.fhwa.saxton.carma.guidance.GuidanceState;
import gov.dot.fhwa.saxton.carma.guidance.GuidanceStateMachine;
import gov.dot.fhwa.saxton.carma.guidance.arbitrator.Arbitrator;
import gov.dot.fhwa.saxton.carma.guidance.plugins.IPlugin;
import gov.dot.fhwa.saxton.carma.guidance.plugins.PluginManager;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.*;
import gov.dot.fhwa.saxton.carma.guidance.util.ILogger;
import gov.dot.fhwa.saxton.carma.guidance.util.LoggerManager;

/**
 * Base class for all Guidance components.
 * <p>
 * Defines the execution framework within the context of both Guidance and the overall system's state
 * due to driver initialization and user command.
 */
public class MobilityRouter extends GuidanceComponent implements IMobilityRouter {

    private final String componentName = "MobilityRouter";
    //private IPublisher<MobilityRequest> bsmPublisher;
    private ISubscriber<MobilityRequest> requestSub;
    private ISubscriber<MobilityAck> ackSub;
    private ISubscriber<MobilityOperation> operationSub;
    private ISubscriber<MobilityPath> pathSub;
    private ConcurrentHashMap<String, LinkedList<MobilityRequestHandler>> requestMap = new ConcurrentHashMap<>();
    private ConcurrentHashMap<String, LinkedList<MobilityAckHandler>> ackMap = new ConcurrentHashMap<>();
    private ConcurrentHashMap<String, LinkedList<MobilityOperationHandler>> operationMap = new ConcurrentHashMap<>();
    private ConcurrentHashMap<String, LinkedList<MobilityPathHandler>> pathMap = new ConcurrentHashMap<>();

    private PluginManager pluginManager;
    private Arbitrator arbitrator;
    private String defaultConflictHandlerName = "";
    private IPlugin defaultConflictHandler;
    private ICollisionDetectionAlgo collisionDetectionAlgo;

    public MobilityRouter(GuidanceStateMachine stateMachine, IPubSubService pubSubService, ConnectedNode node) {
        super(stateMachine, pubSubService, node);
    }

    public void setArbitrator(Arbitrator arbitrator) {
        this.arbitrator = arbitrator;
    }

    public void setPluginManager(PluginManager pluginManager) {
        this.pluginManager = pluginManager;
    }

    @Override
    public String getComponentName() {
        return componentName;
    }

    @Override
    public void onStartup() {
        log.info("Setting up subscribers");
        requestSub = pubSubService.getSubscriberForTopic("incoming_mobility_request", MobilityRequest._TYPE);
        ackSub = pubSubService.getSubscriberForTopic("incoming_mobility_ack", MobilityAck._TYPE);
        operationSub = pubSubService.getSubscriberForTopic("incoming_mobility_operation", MobilityOperation._TYPE);
        pathSub = pubSubService.getSubscriberForTopic("incoming_mobility_path", MobilityPath._TYPE);

        requestSub.registerOnMessageCallback(this::handleMobilityRequest);
        ackSub.registerOnMessageCallback(this::handleMobilityAck);
        operationSub.registerOnMessageCallback(this::handleMobilityOperation);
        pathSub.registerOnMessageCallback(this::handleMobilityPath);

        defaultConflictHandlerName = node.getParameterTree().getString("~default_mobility_conflict_handler", "Yield Plugin");

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
            }
        }

        if (defaultConflictHandler == null) {
            log.warn("No default conflict handling strategy available! Platform will fail on first default conflict!!!");
        }
    }

    @Override
    public void onActive() {

    }

    @Override
    public void onEngaged() {

    }

    @Override
    public void onCleanRestart() {

    }

    @Override
    public void onDeactivate() {

    }
    
    private boolean isBroadcast(MobilityHeader header) {
        return header.getSenderId().equals("");
    }

    private void fireMobilityRequestCallback(MobilityRequestHandler handler, MobilityRequest msg, boolean hasConflict, double conflictStartDist, double conflictEndDist, double conflictStartTime, double conflictEndTime) {
        new Thread(() -> {
            MobilityRequestResponse resp = handler.handleMobilityRequestMessage(msg, hasConflict, conflictStartDist, conflictEndDist, conflictStartTime, conflictEndTime);
            if (resp == MobilityRequestResponse.ACK) {
                collisionDetectionAlgo.addPath(msg);
            } else if (isBroadcast(msg.getHeader()) && resp == MobilityRequestResponse.NO_RESPONSE) {
                collisionDetectionAlgo.addPath(msg);
            } // else, we've rejected it so ignore it
        },
        "MobilityRequestHandlerCallback:" + handler.getClass().getSimpleName()).start();
    }

    private void fireMobilityAckCallback(MobilityAckHandler handler, MobilityAck msg) {
        new Thread(() -> handler.handleMobilityAckMessage(msg),
                "MobilityAckHandlerCallback:" + handler.getClass().getSimpleName()).start();
    }

    private void fireMobilityOperationCallback(MobilityOperationHandler handler, MobilityOperation msg) {
        new Thread(() -> handler.handleMobilityOperationMessage(msg),
                "MobilityOperationHandlerCallback:" + handler.getClass().getSimpleName()).start();
    }

    private void fireMobilityPathCallback(MobilityPathHandler handler, MobilityPath msg, boolean hasConflict,
			double startDist, double endDist, double startTime, double endTime) {
        new Thread(() -> {
            handler.handleMobilityPathMessageWithConflict(msg, hasConflict, startDist, endDist, startTime, endTime);
        },
        "MobilityRequestHandlerCallback:" + handler.getClass().getSimpleName()).start();
	}


    private void handleMobilityRequest(MobilityRequest msg) {
        ConflictSpace conflictSpace = collisionDetectionAlgo.checkConflict(msg.getTrajectoryStartLocation(),
                msg.getOffsets());

        if (conflictSpace.hasConflict()) {
            boolean conflictHandled = false;
            for (Entry<String, LinkedList<MobilityRequestHandler>> entry : requestMap.entrySet()) {
                if (entry.getKey().endsWith(msg.getStrategy())) {
                    for (MobilityRequestHandler handler : entry.getValue()) {
                        fireMobilityRequestCallback(handler, msg, true, conflictSpace.getStartDist(), conflictSpace.getEndDist(), conflictSpace.getStartTime(), conflictSpace.getEndTime());
                        conflictHandled = true;
                    }
                }
            }

            if (!conflictHandled && defaultConflictHandler != null) {
                // Handle in default conflict handler
                fireMobilityRequestCallback(((MobilityRequestHandler) defaultConflictHandler), msg, true, conflictSpace.getStartDist(), conflictSpace.getEndDist(), conflictSpace.getStartTime(), conflictSpace.getEndTime());
            } else {
                throw new RosRuntimeException("Unhandled mobility path conflict detected and no default conflict handler available!!!");
            }
        }
    }

    private void handleMobilityAck(MobilityAck msg) {
        for (Entry<String, LinkedList<MobilityAckHandler>> entry : ackMap.entrySet()) {
            if (entry.getKey().endsWith(msg.getVerificationCode())) {
                for (MobilityAckHandler handler : entry.getValue()) {
                    fireMobilityAckCallback(handler, msg);
                }
            }
        }
    }

    private void handleMobilityOperation(MobilityOperation msg) {
        for (Entry<String, LinkedList<MobilityOperationHandler>> entry : operationMap.entrySet()) {
            if (entry.getKey().endsWith(msg.getStrategy())) {
                for (MobilityOperationHandler handler : entry.getValue()) {
                    fireMobilityOperationCallback(handler, msg);
                }
            }
        }
    }

    private void handleMobilityPath(MobilityPath msg) {
        collisionDetectionAlgo.addPath(msg);
        ConflictSpace conflictSpace = collisionDetectionAlgo.checkConflict(msg.getLocation(),
                msg.getOffsets());

        if (conflictSpace.hasConflict()) {
            // Handle in default conflict handler
            fireMobilityPathCallback(((MobilityPathHandler) defaultConflictHandler), msg, true, conflictSpace.getStartDist(), conflictSpace.getEndDist(), conflictSpace.getStartTime(), conflictSpace.getEndTime());
        }
    }

	public void registerMobilityRequestHandler(String strategyId, MobilityRequestHandler handler) {
        if (!requestMap.containsKey(strategyId)) {
            requestMap.put(strategyId, new LinkedList<MobilityRequestHandler>());
        }
        requestMap.get(strategyId).add(handler);
    }

    public void registerMobilityAckHandler(String strategyId, MobilityAckHandler handler) {
        if (!ackMap.containsKey(strategyId)) {
            ackMap.put(strategyId, new LinkedList<MobilityAckHandler>());
        }
        ackMap.get(strategyId).add(handler);
    }

    public void registerMobilityOperationHandler(String strategyId, MobilityOperationHandler handler) {
        if (!operationMap.containsKey(strategyId)) {
            operationMap.put(strategyId, new LinkedList<MobilityOperationHandler>());
        }
        operationMap.get(strategyId).add(handler);
    }

    public void registerMobilityStatusHandler(String strategyId, MobilityPathHandler handler) {
        if (!pathMap.containsKey(strategyId)) {
            pathMap.put(strategyId, new LinkedList<MobilityPathHandler>());
        }
        pathMap.get(strategyId).add(handler);
    }
}
