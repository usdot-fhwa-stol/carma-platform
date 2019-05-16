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

package gov.dot.fhwa.saxton.carma.plugins.cooperativemerge;

import static org.junit.Assert.assertEquals;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import org.apache.commons.logging.Log;
import org.junit.Before;
import org.junit.Test;
import org.mockito.ArgumentCaptor;
import org.ros.message.MessageFactory;
import org.ros.node.NodeConfiguration;

import cav_msgs.MobilityOperation;
import cav_msgs.MobilityRequest;
import cav_msgs.MobilityResponse;
import gov.dot.fhwa.saxton.carma.guidance.ArbitratorService;
import gov.dot.fhwa.saxton.carma.guidance.IGuidanceCommands;
import gov.dot.fhwa.saxton.carma.guidance.ManeuverPlanner;
import gov.dot.fhwa.saxton.carma.guidance.TrackingService;
import gov.dot.fhwa.saxton.carma.guidance.conflictdetector.IConflictDetector;
import gov.dot.fhwa.saxton.carma.guidance.lightbar.ILightBarManager;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.AccStrategyManager;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.BasicAccStrategyFactory;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.IManeuverInputs;
import gov.dot.fhwa.saxton.carma.guidance.mobilityrouter.IMobilityRouter;
import gov.dot.fhwa.saxton.carma.guidance.params.ParameterSource;
import gov.dot.fhwa.saxton.carma.guidance.plugins.PluginManagementService;
import gov.dot.fhwa.saxton.carma.guidance.plugins.PluginServiceLocator;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.IPubSubService;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.IPublisher;
import gov.dot.fhwa.saxton.carma.guidance.signals.Pipeline;
import gov.dot.fhwa.saxton.carma.guidance.util.ILogger;
import gov.dot.fhwa.saxton.carma.guidance.util.ILoggerFactory;
import gov.dot.fhwa.saxton.carma.guidance.util.LoggerManager;
import gov.dot.fhwa.saxton.carma.guidance.util.RouteService;
import gov.dot.fhwa.saxton.carma.guidance.util.trajectoryconverter.ITrajectoryConverter;
import gov.dot.fhwa.saxton.carma.route.FileStrategy;
import gov.dot.fhwa.saxton.carma.route.Route;

// This test only focus on the behavior of ICooperativeMergeState API.
public class ExecutionStateTest {

    protected CooperativeMergePlugin     mockPlugin;
    protected ILogger                    mockLog;
    protected Log                        mockSimpleLog;
    protected RouteService               mockRouteService;
    protected IManeuverInputs            mockInputs;
    protected IGuidanceCommands          mockCommands;
    protected IMobilityRouter            mockRouter;
    protected PluginServiceLocator       pluginServiceLocator;
    protected Route                      route;
    protected IPublisher<MobilityRequest> mobilityRequestPub;
    protected IPublisher<MobilityOperation> mobilityOperationPub;
    protected IPublisher<MobilityResponse> mobilityResponsePub;
    protected ITrajectoryConverter mockTrajectoryConverter;
    protected ArbitratorService mockArbitratorService;
    protected ManeuverPlanner mockManeuverPlanner;

    NodeConfiguration nodeConfiguration = NodeConfiguration.newPrivate();
    MessageFactory messageFactory = nodeConfiguration.getTopicMessageFactory();

    final String VEHICLE_ID = "veh_id";
    final String BROADCAST_ID = "";
    final String RSU_ID = "rsu_id";
    
    @Before
    public void setup() {
        mockPlugin              = mock(CooperativeMergePlugin.class);
        mockLog                 = mock(ILogger.class);
        mockSimpleLog           = mock(Log.class);
        mockRouteService        = mock(RouteService.class);
        mockInputs              = mock(IManeuverInputs.class);
        mockRouter              = mock(IMobilityRouter.class);
        mockTrajectoryConverter = mock(ITrajectoryConverter.class);
        mockArbitratorService   = mock(ArbitratorService.class);
        mockManeuverPlanner     = mock(ManeuverPlanner.class);
        mockCommands            = mock(IGuidanceCommands.class);

        mobilityRequestPub   = (IPublisher<MobilityRequest>) mock(IPublisher.class);
        mobilityOperationPub = (IPublisher<MobilityOperation>) mock(IPublisher.class);
        mobilityResponsePub  = (IPublisher<MobilityResponse>) mock(IPublisher.class);

        when(mobilityRequestPub.newMessage()).thenReturn(messageFactory.newFromType(MobilityRequest._TYPE));
        when(mobilityOperationPub.newMessage()).thenReturn(messageFactory.newFromType(MobilityOperation._TYPE));
        when(mobilityResponsePub.newMessage()).thenReturn(messageFactory.newFromType(MobilityResponse._TYPE));

        when(mockPlugin.getMobilityRequestPub()).thenReturn(mobilityRequestPub);
        when(mockPlugin.getMobilityOperationPub()).thenReturn(mobilityOperationPub);
        when(mockPlugin.getMobilityResponsePub()).thenReturn(mobilityResponsePub);

        when(mockManeuverPlanner.getManeuverInputs()).thenReturn(mockInputs);
        when(mockManeuverPlanner.getGuidanceCommands()).thenReturn(mockCommands);

        pluginServiceLocator = new PluginServiceLocator(mockArbitratorService,    mock(PluginManagementService.class),
                                                        mock(IPubSubService.class),       mock(ParameterSource.class),
                                                        mockManeuverPlanner,      mockRouteService,
                                                        mockRouter,                       mock(IConflictDetector.class),
                                                        mockTrajectoryConverter, mock(ILightBarManager.class),
                                                        mock(TrackingService.class), null, null);
        when(mockPlugin.getCommsTimeoutMS()).thenReturn(500L);

        when(mockPlugin.getVehicleId()).thenReturn(VEHICLE_ID);


        route = (new FileStrategy("../route/src/test/resources/routes/colonial_farm_rd_outbound.yaml", mockSimpleLog)).load();
        route = Route.fromMessage(route.toMessage(messageFactory)); // Assign waypoint ids

        when(mockRouteService.getCurrentRoute()).thenReturn(route);

        ILoggerFactory mockLoggerFactory = mock(ILoggerFactory.class);
        when(mockLoggerFactory.createLoggerForClass(any(Class.class))).thenReturn(mockLog);
        LoggerManager.setLoggerFactory(mockLoggerFactory);
        
        AccStrategyManager.setAccStrategyFactory(new BasicAccStrategyFactory(1.8, 2.5, 0, 0, 1.5, new Pipeline<>()));
    }
    
    @Test
    public void testOnMobilityResponseWithAcceptedPlan() {
        // Init state
        double rampMeterDTD = 100;
        double mergePointDTD = 500;
        double mergeLength = 100;
        double radius = 100;
        RampMeterData rampMeterData = new RampMeterData(RSU_ID, rampMeterDTD, mergePointDTD, mergeLength, radius);

        String planId = "AA-BB";

        CooperativeMergeManeuver mergeManeuver = new CooperativeMergeManeuver(
            mockPlugin,
            mockPlugin.getCooperativeMergeInputs(),
            pluginServiceLocator.getManeuverPlanner().getManeuverInputs(),
            pluginServiceLocator.getManeuverPlanner().getGuidanceCommands(),
            AccStrategyManager.newAccStrategy(),
            0, 
            600,
            0, 
            20,
            2.5);
      

        final ExecutionState executionState = new ExecutionState(mockPlugin, mockLog, pluginServiceLocator, rampMeterData, planId, mergeManeuver);

        mockPlugin.setState(null, executionState);

        MobilityOperation msg = messageFactory.newFromType(MobilityOperation._TYPE);

        msg.getHeader().setRecipientId(VEHICLE_ID);
        msg.getHeader().setSenderId(RSU_ID);
        msg.getHeader().setPlanId(planId);
        
        msg.setStrategyParams(String.format("STATUS|METER_DIST:%.2f,MERGE_DIST:%.2f,SPEED:%.2f", 0.0, 400.0, 10.0));
        msg.setStrategy(CooperativeMergePlugin.MOBILITY_STRATEGY);

        // Setup mocks
        when(mockInputs.getCurrentSpeed()).thenReturn(10.0);
        when(mockInputs.getMaxAccelLimit()).thenReturn(2.5);
        when(mockRouteService.getCurrentCrosstrackDistance()).thenReturn(0.0);
        when(mockRouteService.getCurrentDowntrackDistance()).thenReturn(0.0);
        when(mockRouteService.getCurrentSegmentIndex()).thenReturn(0);


        executionState.onMobilityOperationMessage(msg);

        // Verify set state value. Two times to account for init state
        ArgumentCaptor<ICooperativeMergeState> newState = ArgumentCaptor.forClass(ICooperativeMergeState.class);
        ArgumentCaptor<ICooperativeMergeState> oldState = ArgumentCaptor.forClass(ICooperativeMergeState.class);
        verify(mockPlugin, times(1)).setState(oldState.capture(), newState.capture());

        // Check we are now in the planning state
        assertEquals("ExecutionState", newState.getValue().toString());
    }


    @Test
    public void testOnMobilityOperationMessage() {
        // Init state
        double rampMeterDTD = 100;
        double mergePointDTD = 500;
        double mergeLength = 100;
        double radius = 100;
        RampMeterData rampMeterData = new RampMeterData(RSU_ID, rampMeterDTD, mergePointDTD, mergeLength, radius);
        
        String planId = "AA-BB";

        MobilityOperation msg = messageFactory.newFromType(MobilityOperation._TYPE);
        msg.getHeader().setRecipientId(VEHICLE_ID);
        msg.getHeader().setSenderId(RSU_ID);
        msg.getHeader().setPlanId(planId);
        msg.setStrategy(CooperativeMergePlugin.MOBILITY_STRATEGY);

        CooperativeMergeManeuver mergeManeuver = new CooperativeMergeManeuver(
            mockPlugin,
            mockPlugin.getCooperativeMergeInputs(),
            pluginServiceLocator.getManeuverPlanner().getManeuverInputs(),
            pluginServiceLocator.getManeuverPlanner().getGuidanceCommands(),
            AccStrategyManager.newAccStrategy(),
            0, 
            600,
            0, 
            20,
            2.5);
        
        final ExecutionState executionState = new ExecutionState(mockPlugin, mockLog, pluginServiceLocator, rampMeterData, planId, mergeManeuver);

        mockPlugin.setState(null, executionState);
        
        // Setup mocks
        when(mockInputs.getCurrentSpeed()).thenReturn(10.0);
        when(mockInputs.getMaxAccelLimit()).thenReturn(2.5);
        when(mockRouteService.getCurrentCrosstrackDistance()).thenReturn(0.0);
        when(mockRouteService.getCurrentDowntrackDistance()).thenReturn(0.0);
        when(mockRouteService.getCurrentSegmentIndex()).thenReturn(0);


        ILoggerFactory mockLoggerFactory = mock(ILoggerFactory.class);
        when(mockLoggerFactory.createLoggerForClass(any(Class.class))).thenReturn(mockLog);
        LoggerManager.setLoggerFactory(mockLoggerFactory);

        msg.setStrategyParams(String.format("STATUS|METER_DIST:%.2f,MERGE_DIST:%.2f,SPEED:%.2f", 0.00, 0.00, 0.00));
        executionState.onMobilityOperationMessage(msg);
        verify(mockLog , times(1)).error("Received operation message with bad params. Exception: java.lang.IllegalArgumentException: Invalid type. Expected: COMMAND String: STATUS|METER_DIST:0.00,MERGE_DIST:0.00,SPEED:0.00");

        msg.setStrategyParams(String.format("COMMAND|SPEED:%.2f,ACCEL:%.2f,STEERING_ANGLE:%.2f", 35.1, 2.1, 1.1));
        executionState.onMobilityOperationMessage(msg);
        verify(mockLog , times(1)).error("Received operation message with suspect strategy values. targetSpeed = 35.1, maxAccel = 2.1, targetSteer = 1.1");

        msg.setStrategyParams(String.format("COMMAND|SPEED:%.2f,ACCEL:%.2f,STEERING_ANGLE:%.2f", 0.4, -2.1, -1.1));
        executionState.onMobilityOperationMessage(msg);
        verify(mockLog , times(1)).error("Received operation message with suspect strategy values. targetSpeed = 0.4, maxAccel = -2.1, targetSteer = -1.1");

    }
}
