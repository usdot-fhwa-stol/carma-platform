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
import java.util.concurrent.ConcurrentHashMap;
import org.ros.node.ConnectedNode;
import cav_msgs.MobilityAck;
import gov.dot.fhwa.saxton.carma.guidance.GuidanceComponent;
import gov.dot.fhwa.saxton.carma.guidance.GuidanceExceptionHandler;
import gov.dot.fhwa.saxton.carma.guidance.GuidanceState;
import gov.dot.fhwa.saxton.carma.guidance.GuidanceStateMachine;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.*;
import gov.dot.fhwa.saxton.carma.guidance.util.ILogger;
import gov.dot.fhwa.saxton.carma.guidance.util.LoggerManager;

/**
 * Base class for all Guidance components.
 * <p>
 * Defines the execution framework within the context of both Guidance and the overall system's state
 * due to driver initialization and user command.
 */
public abstract class MobilityRouter extends GuidanceComponent {
    
    private final String componentName = "MobilityRouter";
    private final Integer YIELD_PLUGIN_ENUM_TYPE = 1; // LANE_CHANGE
    //private IPublisher<MobilityRequest> bsmPublisher;
    private ISubscriber<MobilityRequest> requestSub;
    private ISubscriber<MobilityAck> ackSub;
    private ISubscriber<MobilityOperation> operationSub;
    private ISubscriber<MobilityStatus> statusSub;
    private ConcurrentHashMap<Integer, LinkedList<MobilityRequestHandler>> requestMap = new ConcurrentHashMap<>();
    private ConcurrentHashMap<Integer, LinkedList<MobilityAckHandler>> ackMap = new ConcurrentHashMap<>();
    private ConcurrentHashMap<Integer, LinkedList<MobilityOperationHandler>> operationMap = new ConcurrentHashMap<>();
    private ConcurrentHashMap<Integer, LinkedList<MobilityStatusHandler>> statusMap = new ConcurrentHashMap<>();

    public MobilityRouter(GuidanceStateMachine stateMachine, IPubSubService pubSubService, ConnectedNode node) {
        super(stateMachine, pubSubService, node);
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

        requestSub.registerOnMessageCallback((MobilityRequest msg) -> {
            handleMobilityRequest(msg);
        });

        ackSub.registerOnMessageCallback((MobilityAck msg) -> {
            handleMobilityAck(msg);
        });

        operationSub.registerOnMessageCallback((MobilityOperation msg) -> {
            handleMobilityOperation(msg);
        });

        statusSub.registerOnMessageCallback((MobilityStatus msg) -> {
            handleMobilityStatus(msg);
        });
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

    private void handleMobilityRequest(MobilityRequest msg) {

        // TODO add path to data structure
        addToCollisionTree(msg.path);
        List<MobilityRequestHandler> handlers = operationMap.get(msg.target);
        if (handlers == null) { // TODO Might be unsafe to leave without ensuring replan on conflict
            return;
        }

        // TODO make call for conflict space
        ConflictSpace conflictSpace = getConflictSpace();
        for (MobilityRequestHandler handler: handlers) {
            // Plugin can handle this kind of message and it has a conflict. Allow it to handle the message
            if (conflictSpace.hasConflict()) {
                if (conflictSpace.conflictsWithPlugin(msg.targetPlugin)) {
                    handler.handleMessage(msg, true, conflictSpace.startDist, conflictSpace.endDist, conflictSpace.startTime, conflictSpace.endTime);
                } else {
                    // TODO route to yield plugin
                }
            }
            handler.handleMessage(msg, false, conflictSpace.startDist, conflictSpace.endDist, conflictSpace.startTime, conflictSpace.endTime);
        }
    }

    private void handleMobilityAck(MobilityAck msg) {
        List<MobilityAckHandler> handlers = operationMap.get(msg.target);
        if (handlers == null) {
            return;
        }
        for (MobilityAckHandler handler: handlers) {
            handler.handleMessage(msg);
        }
    }

    private void handleMobilityOperation(MobilityOperation msg) {
        List<MobilityOperationHandler> handlers = operationMap.get(msg.target);
        if (handlers == null) {
            return;
        }
        for (MobilityOperationHandler handler: handlers) {
            handler.handleMessage(msg);
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
                    handler.handleMessage(msg, true, conflictSpace.startDist, conflictSpace.endDist, conflictSpace.startTime, conflictSpace.endTime);
                } else {
                    // TODO route to yield plugin
                }
            }
            handler.handleMessage(msg, false, conflictSpace.startDist, conflictSpace.endDist, conflictSpace.startTime, conflictSpace.endTime);
        }
    }

    public void registerMobilityRequestHandler(Integer targetPlugin, MobilityRequestHandler handler) {
        if (!requestMap.containsKey(targetPlugin)) {
            requestMap.put(targetPlugin, new LinkedList<MobilityRequestHandler>());
        }
        requestMap.get(targetPlugin).add(handler);
    }

    public void registerMobilityAckHandler(Integer targetPlugin, MobilityAckHandler handler) {
        if (!ackMap.containsKey(targetPlugin)) {
            ackMap.put(targetPlugin, new LinkedList<MobilityAckHandler>());
        }
        ackMap.get(targetPlugin).add(handler);
    }

    public void registerMobilityOperationHandler(Integer targetPlugin, MobilityOperationHandler handler) {
        if (!operationMap.containsKey(targetPlugin)) {
            operationMap.put(targetPlugin, new LinkedList<MobilityOperationHandler>());
        }
        operationMap.get(targetPlugin).add(handler);
    }

    public void registerMobilityStatusHandler(Integer targetPlugin, MobilityStatusHandler handler) {
        if (!statusMap.containsKey(targetPlugin)) {
            statusMap.put(targetPlugin, new LinkedList<MobilityStatusHandler>());
        }
        statusMap.get(targetPlugin).add(handler);
    }
}
