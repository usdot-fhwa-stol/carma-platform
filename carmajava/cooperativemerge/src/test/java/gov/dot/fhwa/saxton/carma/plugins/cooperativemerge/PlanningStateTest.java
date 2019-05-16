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
import static org.mockito.ArgumentMatchers.*;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import java.util.SortedSet;
import java.util.TreeSet;

import static org.mockito.Mockito.doAnswer;

import org.apache.commons.logging.Log;
import org.junit.Before;
import org.junit.Test;
import org.mockito.ArgumentCaptor;
import org.mockito.invocation.InvocationOnMock;
import org.mockito.stubbing.Answer;
import org.ros.message.MessageFactory;
import org.ros.node.NodeConfiguration;

import cav_msgs.MobilityOperation;
import cav_msgs.MobilityRequest;
import cav_msgs.MobilityResponse;
import gov.dot.fhwa.saxton.carma.guidance.ArbitratorService;
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
import gov.dot.fhwa.saxton.carma.guidance.util.SpeedLimit;
import gov.dot.fhwa.saxton.carma.guidance.util.trajectoryconverter.ITrajectoryConverter;
import gov.dot.fhwa.saxton.carma.route.FileStrategy;
import gov.dot.fhwa.saxton.carma.route.Route;
import gov.dot.fhwa.saxton.carma.guidance.trajectory.Trajectory;

// This test only focus on the behavior of ICooperativeMergeState API.
public class PlanningStateTest {

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
    protected ArbitratorService mockArbitratorService;
    protected ManeuverPlanner mockManeuverPlanner;

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
        mockArbitratorService = mock(ArbitratorService.class);
        mockManeuverPlanner = mock(ManeuverPlanner.class);

        mobilityRequestPub = (IPublisher<MobilityRequest>) mock(IPublisher.class);
        mobilityOperationPub = (IPublisher<MobilityOperation>) mock(IPublisher.class);
        mobilityResponsePub = (IPublisher<MobilityResponse>) mock(IPublisher.class);

        when(mobilityRequestPub.newMessage()).thenReturn(messageFactory.newFromType(MobilityRequest._TYPE));
        when(mobilityOperationPub.newMessage()).thenReturn(messageFactory.newFromType(MobilityOperation._TYPE));
        when(mobilityResponsePub.newMessage()).thenReturn(messageFactory.newFromType(MobilityResponse._TYPE));

        when(mockPlugin.getMobilityRequestPub()).thenReturn(mobilityRequestPub);
        when(mockPlugin.getMobilityOperationPub()).thenReturn(mobilityOperationPub);
        when(mockPlugin.getMobilityResponsePub()).thenReturn(mobilityResponsePub);

        when(mockManeuverPlanner.getManeuverInputs()).thenReturn(mockInputs);

        pluginServiceLocator = new PluginServiceLocator(mockArbitratorService,    mock(PluginManagementService.class),
                                                        mock(IPubSubService.class),       mock(ParameterSource.class),
                                                        mockManeuverPlanner,      mockRouteService,
                                                        mockRouter,                       mock(IConflictDetector.class),
                                                        mockTrajectoryConverter, mock(ILightBarManager.class),
                                                        mock(TrackingService.class), null, null);
        when(mockPlugin.getCommsTimeoutMS()).thenReturn(500L);

        when(mockPlugin.getVehicleId()).thenReturn(VEHICLE_ID);
        final StandbyState standbyState = new StandbyState(mockPlugin, mockLog, pluginServiceLocator);
        mockPlugin.setState(null, standbyState);
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

        when(mockTrajectoryConverter.pathToMessage(any())).thenReturn(messageFactory.newFromType(cav_msgs.Trajectory._TYPE));

        final PlanningState planningState = new PlanningState(mockPlugin, mockLog, pluginServiceLocator, rampMeterData);
        final Trajectory traj =  new Trajectory(0, 650);

        mockPlugin.setState(null, planningState);

        MobilityResponse msg = messageFactory.newFromType(MobilityResponse._TYPE);

        msg.getHeader().setRecipientId(VEHICLE_ID);
        msg.getHeader().setSenderId(RSU_ID);
        msg.getHeader().setPlanId(planningState.planId);
        msg.setIsAccepted(true);

        // Setup mocks
        when(mockInputs.getCurrentSpeed()).thenReturn(10.0);
        when(mockInputs.getMaxAccelLimit()).thenReturn(2.5);
        when(mockRouteService.getCurrentCrosstrackDistance()).thenReturn(0.0);
        when(mockRouteService.getCurrentDowntrackDistance()).thenReturn(0.0);
        when(mockRouteService.getCurrentSegmentIndex()).thenReturn(0);
        when(mockRouteService.isAlgorithmEnabledInRange(traj.getStartLocation(), traj.getEndLocation(), CooperativeMergePlugin.COOPERATIVE_MERGE_FLAG))
        .thenReturn(true);

        SortedSet<SpeedLimit> limits = new TreeSet<>((a, b) -> Double.compare(a.getLocation(), b.getLocation()));
        limits.add(new SpeedLimit(650, 20));

        when(mockRouteService.getSpeedLimitsInRange(anyDouble(), anyDouble())).thenReturn(limits);

        doAnswer(new Answer() {
            public Object answer(InvocationOnMock invocation) {
                planningState.planTrajectory(traj, 10.0);
                return null;
            }}).when(mockArbitratorService).requestNewPlan();

        planningState.onMobilityResponseMessage(msg);

        // Verify set state value. Two times to account for init state
        ArgumentCaptor<ICooperativeMergeState> newState = ArgumentCaptor.forClass(ICooperativeMergeState.class);
        ArgumentCaptor<ICooperativeMergeState> oldState = ArgumentCaptor.forClass(ICooperativeMergeState.class);
        verify(mockPlugin, times(2)).setState(oldState.capture(), newState.capture());

        // Check we are now in the planning state
        assertEquals("PlanningState", newState.getValue().toString());
    }
}
