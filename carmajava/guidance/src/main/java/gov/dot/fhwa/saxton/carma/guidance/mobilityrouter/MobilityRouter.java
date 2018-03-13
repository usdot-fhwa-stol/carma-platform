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
import org.ros.node.ConnectedNode;
import cav_msgs.MobilityAck;
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
    private final Integer YIELD_PLUGIN_ENUM_TYPE = 1; // LANE_CHANGE
    //private IPublisher<MobilityRequest> bsmPublisher;
    private ISubscriber<MobilityRequest> requestSub;
    private ISubscriber<MobilityAck> ackSub;
    private ISubscriber<MobilityOperation> operationSub;
    private ISubscriber<MobilityStatus> statusSub;
    private ConcurrentHashMap<String, LinkedList<MobilityRequestHandler>> requestMap = new ConcurrentHashMap<>();
    private ConcurrentHashMap<String, LinkedList<MobilityAckHandler>> ackMap = new ConcurrentHashMap<>();
    private ConcurrentHashMap<String, LinkedList<MobilityOperationHandler>> operationMap = new ConcurrentHashMap<>();
    private ConcurrentHashMap<String, LinkedList<MobilityStatusHandler>> statusMap = new ConcurrentHashMap<>();

    private PluginManager pluginManager;
    private Arbitrator arbitrator;
    private IPlugin yieldPlugin;

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
        statusSub = pubSubService.getSubscriberForTopic("incoming_mobility_status", MobilityStatus._TYPE);

        requestSub.registerOnMessageCallback(this::handleMobilityRequest);
        ackSub.registerOnMessageCallback(this::handleMobilityAck);
        operationSub.registerOnMessageCallback(this::handleMobilityOperation);
        statusSub.registerOnMessageCallback(this::handleMobilityStatus);
        log.info("Setup complete");
    }
    
    @Override
    public void onSystemReady() {

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

    private void fireMobilityRequestCallback(MobilityOperationHandler handler, MobilityRequest msg) {
        new Thread(() -> handler.handleMobilityOperationMessage(msg), 
        "MobilityRequestHandlerCallback:" + handler.getClass().getSimpleName())
        .start();
    }

    private void fireMobilityAckCallback(MobilityAckHandler handler, MobilityAck msg) {
        new Thread(() -> handler.handleMobilityAckMessage(msg), 
        "MobilityAckHandlerCallback:" + handler.getClass().getSimpleName())
        .start();
    }

    private void fireMobilityOperationCallback(MobilityOperationHandler handler, MobilityOperation msg) {
        new Thread(() -> handler.handleMobilityOperationMessage(msg), 
        "MobilityOperationHandlerCallback:" + handler.getClass().getSimpleName())
        .start();
    }

    private void handleMobilityRequest(MobilityRequest msg) {
        // TODO add path to data structure
        addToCollisionTree(msg.path);

        for (Entry<String, LinkedList<MobilityRequestHandler>> entry : requestMap.entrySet()) {
            if (entry.getKey().endsWith(msg.getStrategy())) {
                for (MobilityRequestHandler handler : entry.getValue()) {
                    fireMobilityRequestCallback(handler, msg);
                }
            }
        }
        
        List<MobilityRequestHandler> handlers = operationMap.get(msg.target);
        if (handlers == null) { // TODO Might be unsafe to leave without ensuring replan on conflict
            return;
        }

        // TODO make call for conflict space
        ConflictSpace conflictSpace = getConflictSpace();
        for (MobilityRequestHandler handler : handlers) {
            // Plugin can handle this kind of message and it has a conflict. Allow it to handle the message
            if (conflictSpace.hasConflict()) {
                if (conflictSpace.conflictsWithPlugin(msg.targetPlugin)) {
                    handler.handleMessage(msg, true, conflictSpace.startDist, conflictSpace.endDist, conflictSpace.startTime, conflictSpace.endTime);
                } else {
                    // TODO route to yield plugin
                }
            }
            handler.handleMobilityRequestMessage(msg, false, conflictSpace.startDist, conflictSpace.endDist, conflictSpace.startTime, conflictSpace.endTime);
        }
    }

    private void handleMobilityAck(MobilityAck msg) {
        for (Entry<String, LinkedList<MobilityAckHandler>> entry : requestMap.entrySet()) {
            if (entry.getKey().endsWith(msg.getStrategy())) {
                for (MobilityAckHandler handler : entry.getValue()) {
                    fireMobilityAckCallback(handler, msg);
                }
            }
        }
    }

    private void handleMobilityOperation(MobilityOperation msg) {
        for (Entry<String, LinkedList<MobilityOperationHandler>> entry : requestMap.entrySet()) {
            if (entry.getKey().endsWith(msg.getStrategy())) {
                for (MobilityOperationHandler handler : entry.getValue()) {
                    fireMobilityOperationCallback(handler, msg);
                }
            }
        }
    }

    private void handleMobilityStatus(MobilityStatus msg) {

        // TODO add path to data structure
        addToCollisionTree(msg.path);
        List<MobilityStatusHandler> handlers = statusMap.get(msg.target);
        if (handlers == null) { // TODO Might be unsafe to leave without ensuring replan on conflict
            return;
        }

        // TODO make call for conflict space
        ConflictSpace conflictSpace = getConflictSpace();
        for (MobilityStatusHandler handler: handlers) {
            // Plugin can handle this kind of message and it has a conflict. Allow it to handle the message
            if (conflictSpace.hasConflict()) {
                if (conflictSpace.conflictsWithPlugin(msg.targetPlugin)) {
                    handler.handleMobilityStatusMessage(msg, true, conflictSpace.startDist, conflictSpace.endDist, conflictSpace.startTime, conflictSpace.endTime);
                } else {
                    // TODO route to yield plugin
                }
            }
            handler.handleMobilityStatusMessage(msg, false, conflictSpace.startDist, conflictSpace.endDist, conflictSpace.startTime, conflictSpace.endTime);
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

    public void registerMobilityStatusHandler(String strategyId, MobilityStatusHandler handler) {
        if (!statusMap.containsKey(strategyId)) {
            statusMap.put(strategyId, new LinkedList<MobilityStatusHandler>());
        }
        statusMap.get(strategyId).add(handler);
    }
}
