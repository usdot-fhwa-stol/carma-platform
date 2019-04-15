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

package gov.dot.fhwa.saxton.carma.rsumetering;

import java.util.List;

import static org.junit.Assert.assertEquals;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;
import static org.mockito.Matchers.any;
import static org.mockito.Matchers.anyString;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.apache.commons.logging.Log;
import org.junit.Before;
import org.junit.Test;
import org.mockito.ArgumentCaptor;
import org.ros.message.MessageFactory;
import org.ros.node.NodeConfiguration;


import cav_msgs.MobilityOperation;
import cav_msgs.MobilityRequest;
import cav_msgs.MobilityResponse;
import gov.dot.fhwa.saxton.carma.rosutils.MobilityHelper;
import gov.dot.fhwa.saxton.carma.rosutils.SaxtonLogger;
import gov.dot.fhwa.saxton.carma.guidance.util.ILogger;
import gov.dot.fhwa.saxton.carma.guidance.util.ILoggerFactory;
import gov.dot.fhwa.saxton.carma.guidance.util.LoggerManager;

// This test only focus on the behavior of CommandingState API.
public class CommandingStateTest {
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
    public void testOnMobilityOperationMessage() {
        assertEquals(2,1);
    }


}