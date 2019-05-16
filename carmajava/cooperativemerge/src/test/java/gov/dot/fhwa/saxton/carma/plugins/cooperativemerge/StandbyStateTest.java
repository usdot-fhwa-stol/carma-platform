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
import org.ros.rosjava_geometry.Transform;

import cav_msgs.MobilityOperation;
import cav_msgs.MobilityRequest;
import cav_msgs.MobilityResponse;
import gov.dot.fhwa.saxton.carma.geometry.GeodesicCartesianConverter;
import gov.dot.fhwa.saxton.carma.geometry.cartesian.Point3D;
import gov.dot.fhwa.saxton.carma.geometry.geodesic.Location;
import gov.dot.fhwa.saxton.carma.guidance.ArbitratorService;
import gov.dot.fhwa.saxton.carma.guidance.ManeuverPlanner;
import gov.dot.fhwa.saxton.carma.guidance.TrackingService;
import gov.dot.fhwa.saxton.carma.guidance.conflictdetector.IConflictDetector;
import gov.dot.fhwa.saxton.carma.guidance.lightbar.ILightBarManager;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.IManeuverInputs;
import gov.dot.fhwa.saxton.carma.guidance.mobilityrouter.IMobilityRouter;
import gov.dot.fhwa.saxton.carma.guidance.params.ParameterSource;
import gov.dot.fhwa.saxton.carma.guidance.plugins.PluginManagementService;
import gov.dot.fhwa.saxton.carma.guidance.plugins.PluginServiceLocator;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.IPubSubService;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.IPublisher;
import gov.dot.fhwa.saxton.carma.guidance.util.ILogger;
import gov.dot.fhwa.saxton.carma.guidance.util.ILoggerFactory;
import gov.dot.fhwa.saxton.carma.guidance.util.LoggerManager;
import gov.dot.fhwa.saxton.carma.guidance.util.RouteService;
import gov.dot.fhwa.saxton.carma.guidance.util.trajectoryconverter.ITrajectoryConverter;
import gov.dot.fhwa.saxton.carma.route.FileStrategy;
import gov.dot.fhwa.saxton.carma.route.Route;

// This test only focus on the behavior of ICooperativeMergeState API.
public class StandbyStateTest {

    protected CooperativeMergePlugin     mockPlugin;
    protected ILogger                    mockLog;
    protected Log                        mockSimpleLog;
    protected RouteService               mockRouteService;
    protected IManeuverInputs            mockInputs;
    protected IMobilityRouter            mockRouter;
    protected PluginServiceLocator       pluginServiceLocator;
    protected Route                      route;
    protected IPublisher<MobilityRequest> mobilityRequestPub;
    protected IPublisher<MobilityOperation> mobilityOperationPub;
    protected IPublisher<MobilityResponse> mobilityResponsePub;
    protected ITrajectoryConverter mockTrajectoryConverter;

    NodeConfiguration nodeConfiguration = NodeConfiguration.newPrivate();
    MessageFactory messageFactory = nodeConfiguration.getTopicMessageFactory();

    final String VEHICLE_ID = "veh_id";
    final String BROADCAST_ID = "";
    final String RSU_ID = "rsu_id";
    
    @Before
    public void setup() {
        mockPlugin           = mock(CooperativeMergePlugin.class);
        mockLog              = mock(ILogger.class);
        mockSimpleLog        = mock(Log.class);
        mockRouteService     = mock(RouteService.class);
        mockInputs           = mock(IManeuverInputs.class);
        mockRouter           = mock(IMobilityRouter.class);
        mockTrajectoryConverter = mock(ITrajectoryConverter.class);

        mobilityRequestPub = (IPublisher<MobilityRequest>) mock(IPublisher.class);
        mobilityOperationPub = (IPublisher<MobilityOperation>) mock(IPublisher.class);
        mobilityResponsePub = (IPublisher<MobilityResponse>) mock(IPublisher.class);

        when(mobilityRequestPub.newMessage()).thenReturn(messageFactory.newFromType(MobilityRequest._TYPE));
        when(mobilityOperationPub.newMessage()).thenReturn(messageFactory.newFromType(MobilityOperation._TYPE));
        when(mobilityResponsePub.newMessage()).thenReturn(messageFactory.newFromType(MobilityResponse._TYPE));

        when(mockPlugin.getMobilityRequestPub()).thenReturn(mobilityRequestPub);
        when(mockPlugin.getMobilityOperationPub()).thenReturn(mobilityOperationPub);
        when(mockPlugin.getMobilityResponsePub()).thenReturn(mobilityResponsePub);

        pluginServiceLocator = new PluginServiceLocator(mock(ArbitratorService.class),    mock(PluginManagementService.class),
                                                        mock(IPubSubService.class),       mock(ParameterSource.class),
                                                        mock(ManeuverPlanner.class),      mockRouteService,
                                                        mockRouter,                       mock(IConflictDetector.class),
                                                        mockTrajectoryConverter,          mock(ILightBarManager.class),
                                                        mock(TrackingService.class), null, null);
        when(mockPlugin.getCommsTimeoutMS()).thenReturn(500L);

        when(mockPlugin.getVehicleId()).thenReturn(VEHICLE_ID);


        route = (new FileStrategy("../route/src/test/resources/routes/colonial_farm_rd_outbound.yaml", mockSimpleLog)).load();
        route = Route.fromMessage(route.toMessage(messageFactory)); // Assign waypoint ids

        when(mockRouteService.getCurrentRoute()).thenReturn(route);
    }
    
    @Test
    public void testOnMobilityRequestWithCloseRSU() {
        // Init state
        StandbyState standbyState = new StandbyState(mockPlugin, mockLog, pluginServiceLocator);
        mockPlugin.setState(null, standbyState);

        when(mockTrajectoryConverter.pathToMessage(any())).thenReturn(messageFactory.newFromType(cav_msgs.Trajectory._TYPE));

        MobilityRequest msg = messageFactory.newFromType(MobilityRequest._TYPE);

        msg.getHeader().setRecipientId(BROADCAST_ID);
        msg.getHeader().setSenderId(RSU_ID);
        msg.setStrategy(CooperativeMergePlugin.MOBILITY_STRATEGY);
        double radius = 100;
        double mergeDist = 500;
        double mergeLength = 100;
        String params = String.format("INFO|RADIUS:%.2f,MERGE_DIST:%.2f,MERGE_LENGTH:%.2f", radius, mergeDist, mergeLength);
        msg.setStrategyParams(params);

        GeodesicCartesianConverter gcc = new GeodesicCartesianConverter();

        Point3D currentECEF = gcc.geodesic2Cartesian(new Location(38.95647, -77.15031, 72.0) , Transform.identity());
        msg.getLocation().setEcefX((int)(currentECEF.getX() * StandbyState.CM_PER_M));
        msg.getLocation().setEcefY((int)(currentECEF.getY() * StandbyState.CM_PER_M));
        msg.getLocation().setEcefZ((int)(currentECEF.getZ() * StandbyState.CM_PER_M));
        standbyState.onMobilityRequestMessage(msg);

        // Verify set state value. Two times to account for init state
        ArgumentCaptor<ICooperativeMergeState> newState = ArgumentCaptor.forClass(ICooperativeMergeState.class);
        ArgumentCaptor<ICooperativeMergeState> oldState = ArgumentCaptor.forClass(ICooperativeMergeState.class);
        verify(mockPlugin, times(2)).setState(oldState.capture(), newState.capture());

        // Check we are now in the planning state
        assertEquals("PlanningState", newState.getValue().toString());
    }

    @Test
    public void testOnMobilityRequestWithFarRSU() {
        // Init state
        StandbyState standbyState = new StandbyState(mockPlugin, mockLog, pluginServiceLocator);
        mockPlugin.setState(null, standbyState);

        when(mockTrajectoryConverter.pathToMessage(any())).thenReturn(messageFactory.newFromType(cav_msgs.Trajectory._TYPE));

        MobilityRequest msg = messageFactory.newFromType(MobilityRequest._TYPE);

        msg.getHeader().setRecipientId(BROADCAST_ID);
        msg.getHeader().setSenderId(RSU_ID);
        msg.setStrategy(CooperativeMergePlugin.MOBILITY_STRATEGY);
        double radius = 100;
        double mergeDist = 500;
        double mergeLength = 100;
        String params = String.format("INFO|RADIUS:%.2f,MERGE_DIST:%.2f,MERGE_LENGTH:%.2f", radius, mergeDist, mergeLength);
        msg.setStrategyParams(params);

        GeodesicCartesianConverter gcc = new GeodesicCartesianConverter();

        Point3D currentECEF = gcc.geodesic2Cartesian(new Location(90, 90, 72.0) , Transform.identity());
        msg.getLocation().setEcefX((int)(currentECEF.getX() * StandbyState.CM_PER_M));
        msg.getLocation().setEcefY((int)(currentECEF.getY() * StandbyState.CM_PER_M));
        msg.getLocation().setEcefZ((int)(currentECEF.getZ() * StandbyState.CM_PER_M));
        standbyState.onMobilityRequestMessage(msg);

        // Verify set state value. Two times to account for init state
        ArgumentCaptor<ICooperativeMergeState> newState = ArgumentCaptor.forClass(ICooperativeMergeState.class);
        ArgumentCaptor<ICooperativeMergeState> oldState = ArgumentCaptor.forClass(ICooperativeMergeState.class);
        verify(mockPlugin, times(1)).setState(oldState.capture(), newState.capture());

        // Check we are now in the planning state
        assertEquals("StandbyState", newState.getValue().toString());
    }

    @Test
    public void testOnMobilityRequestMessage() {
        // Init state
        StandbyState standbyState = new StandbyState(mockPlugin, mockLog, pluginServiceLocator);
        mockPlugin.setState(null, standbyState);

        when(mockTrajectoryConverter.pathToMessage(any())).thenReturn(messageFactory.newFromType(cav_msgs.Trajectory._TYPE));

        MobilityRequest msg = messageFactory.newFromType(MobilityRequest._TYPE);

        msg.getHeader().setRecipientId(BROADCAST_ID);
        msg.getHeader().setSenderId(RSU_ID);
        msg.setStrategy(CooperativeMergePlugin.MOBILITY_STRATEGY);

        ILoggerFactory mockLoggerFactory = mock(ILoggerFactory.class);
        when(mockLoggerFactory.createLoggerForClass(any(Class.class))).thenReturn(mockLog);
        LoggerManager.setLoggerFactory(mockLoggerFactory);

        GeodesicCartesianConverter gcc = new GeodesicCartesianConverter();

        Point3D currentECEF = gcc.geodesic2Cartesian(new Location(90, 90, 72.0) , Transform.identity());
        msg.getLocation().setEcefX((int)(currentECEF.getX() * StandbyState.CM_PER_M));
        msg.getLocation().setEcefY((int)(currentECEF.getY() * StandbyState.CM_PER_M));
        msg.getLocation().setEcefZ((int)(currentECEF.getZ() * StandbyState.CM_PER_M));

        double radius = 100;
        double mergeDist = -10;
        double mergeLength = 5;

        msg.setStrategyParams(String.format("COMMAND|RADIUS:%.2f,MERGE_DIST:%.2f,MERGE_LENGTH:%.2f", radius, mergeDist, mergeLength));
        standbyState.onMobilityRequestMessage(msg);
        verify(mockLog , times(1)).error("Received mobility request with invalid params. Exception: java.lang.IllegalArgumentException: Invalid type. Expected: INFO String: COMMAND|RADIUS:100.00,MERGE_DIST:-10.00,MERGE_LENGTH:5.00");

        radius = 99;
        mergeDist = -11;
        mergeLength = 4;

        msg.setStrategyParams(String.format("INFO|RADIUS:%.2f,MERGE_DIST:%.2f,MERGE_LENGTH:%.2f", radius, mergeDist, mergeLength));
        standbyState.onMobilityRequestMessage(msg);
        verify(mockLog , times(1)).error("Received mobility request with suspect strategy values. meterRadius = " + radius + ", mergeDTDFromMeter = " + mergeDist + ", mergeLength = " + mergeLength);

        radius = 10001;
        mergeDist = 1001;
        mergeLength = 801;

        msg.setStrategyParams(String.format("INFO|RADIUS:%.2f,MERGE_DIST:%.2f,MERGE_LENGTH:%.2f", radius, mergeDist, mergeLength));
        standbyState.onMobilityRequestMessage(msg);
        verify(mockLog , times(1)).error("Received mobility request with suspect strategy values. meterRadius = " + radius + ", mergeDTDFromMeter = " + mergeDist + ", mergeLength = " + mergeLength);

    }

}
