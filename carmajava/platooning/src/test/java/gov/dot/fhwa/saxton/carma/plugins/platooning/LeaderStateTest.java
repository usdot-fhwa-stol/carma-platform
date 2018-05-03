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

package gov.dot.fhwa.saxton.carma.plugins.platooning;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import java.util.concurrent.atomic.AtomicBoolean;

import org.junit.Before;
import org.junit.Test;
import org.mockito.ArgumentCaptor;

import cav_msgs.MobilityHeader;
import cav_msgs.MobilityOperation;
import cav_msgs.MobilityRequest;
import cav_msgs.PlanType;
import gov.dot.fhwa.saxton.carma.guidance.ArbitratorService;
import gov.dot.fhwa.saxton.carma.guidance.ManeuverPlanner;
import gov.dot.fhwa.saxton.carma.guidance.TrackingService;
import gov.dot.fhwa.saxton.carma.guidance.arbitrator.TrajectoryPlanningResponse;
import gov.dot.fhwa.saxton.carma.guidance.conflictdetector.IConflictDetector;
import gov.dot.fhwa.saxton.carma.guidance.lightbar.ILightBarManager;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.IManeuverInputs;
import gov.dot.fhwa.saxton.carma.guidance.mobilityrouter.IMobilityRouter;
import gov.dot.fhwa.saxton.carma.guidance.mobilityrouter.MobilityRequestResponse;
import gov.dot.fhwa.saxton.carma.guidance.params.ParameterSource;
import gov.dot.fhwa.saxton.carma.guidance.plugins.PluginManagementService;
import gov.dot.fhwa.saxton.carma.guidance.plugins.PluginServiceLocator;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.IPubSubService;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.IPublisher;
import gov.dot.fhwa.saxton.carma.guidance.trajectory.Trajectory;
import gov.dot.fhwa.saxton.carma.guidance.util.ILogger;
import gov.dot.fhwa.saxton.carma.guidance.util.RouteService;
import gov.dot.fhwa.saxton.carma.guidance.util.trajectoryconverter.ITrajectoryConverter;

// This test only focus on the behavior of IPlatooningState API.
public class LeaderStateTest {

    private PlatooningPlugin            mockPlugin;
    private ILogger                     mockLog;
    private RouteService                mockRouteService;
    private PlatoonManager              mockManager;
    private IManeuverInputs             mockInputs;
    private IMobilityRouter             mockRouter;
    private PluginServiceLocator        pluginServiceLocator;
    private IPublisher<MobilityRequest> mockRequestPub;
    private IPlatooningState            leaderState;
    
    @Before
    public void setup() {
        mockPlugin           = mock(PlatooningPlugin.class);
        mockLog              = mock(ILogger.class);
        mockRouteService     = mock(RouteService.class);
        mockManager          = mock(PlatoonManager.class);
        mockInputs           = mock(IManeuverInputs.class);
        mockRouter           = mock(IMobilityRouter.class);
        mockRequestPub       = mock(IPublisher.class);
        pluginServiceLocator = new PluginServiceLocator(mock(ArbitratorService.class),    mock(PluginManagementService.class),
                                                        mock(IPubSubService.class),       mock(ParameterSource.class),
                                                        mock(ManeuverPlanner.class),      mockRouteService,
                                                        mockRouter,                       mock(IConflictDetector.class),
                                                        mock(ITrajectoryConverter.class), mock(ILightBarManager.class),
                                                        mock(TrackingService.class));
        mockPlugin.platoonManager = mockManager;
        mockPlugin.maxPlatoonSize = 5;
        mockPlugin.mobilityRequestPublisher = mockRequestPub;
        mockManager.currentPlatoonID = "ABC";
        mockPlugin.handleMobilityPath = new AtomicBoolean(true);
        when(mockPlugin.getManeuverInputs()).thenReturn(mockInputs);
        leaderState = new LeaderState(mockPlugin, mockLog, pluginServiceLocator);
    }
    
    @Test
    public void planTrajectoryWithoutAnyWindow() {
        Trajectory traj = new Trajectory(0, 50);
        when(mockRouteService.isAlgorithmEnabledInRange(0.0, 50.0, PlatooningPlugin.PLATOONING_FLAG)).thenReturn(false);
        TrajectoryPlanningResponse tpr = leaderState.planTrajectory(traj, 0);
        assertTrue(tpr.getRequests().isEmpty());
        assertNull(traj.getComplexManeuver());
        assertTrue(traj.getLongitudinalManeuvers().isEmpty());
        assertTrue(traj.getLateralManeuvers().isEmpty());
        ArgumentCaptor<StandbyState> newState = ArgumentCaptor.forClass(StandbyState.class);
        verify(mockPlugin).setState(newState.capture());
        assertEquals("StandbyState", newState.getValue().toString());
    }
    
    @Test
    public void planTrajectoryWithAnOpenWindow() {
        Trajectory traj = new Trajectory(0, 50);
        when(mockRouteService.isAlgorithmEnabledInRange(0.0, 50.0, PlatooningPlugin.PLATOONING_FLAG)).thenReturn(true);
        TrajectoryPlanningResponse tpr = leaderState.planTrajectory(traj, 0);
        assertTrue(tpr.getRequests().isEmpty());
        assertNull(traj.getComplexManeuver());
        assertTrue(traj.getLongitudinalManeuvers().isEmpty());
        assertTrue(traj.getLateralManeuvers().isEmpty());
        verify(mockPlugin, times(0)).setState(any());
    }
    
    @Test
    public void onUnknownRequest() {
        MobilityRequest unknownRequest = mock(MobilityRequest.class);
        PlanType mockType = mock(PlanType.class);
        when(mockType.getType()).thenReturn(PlanType.UNKNOWN);
        when(unknownRequest.getPlanType()).thenReturn(mockType);
        assertEquals(MobilityRequestResponse.NO_RESPONSE, leaderState.onMobilityRequestMessgae(unknownRequest));
    }
    
    @Test
    public void onJoinRequestWithTooManyMember() {
        MobilityRequest unknownRequest = mock(MobilityRequest.class);
        PlanType mockType = mock(PlanType.class);
        MobilityHeader mockHeader = mock(MobilityHeader.class);
        when(mockType.getType()).thenReturn(PlanType.JOIN_PLATOON_AT_REAR);
        when(mockHeader.getSenderId()).thenReturn("1");
        when(unknownRequest.getPlanType()).thenReturn(mockType);
        when(unknownRequest.getHeader()).thenReturn(mockHeader);
        when(unknownRequest.getStrategyParams()).thenReturn("SIZE:1,SPEED:2.50,DTD:10.50");
        when(mockManager.getTotalPlatooningSize()).thenReturn(5);
        assertEquals(MobilityRequestResponse.NACK, leaderState.onMobilityRequestMessgae(unknownRequest));
    }
    
    @Test
    public void onJoinRequestWithInMaxGap() {
        MobilityRequest unknownRequest = mock(MobilityRequest.class);
        PlanType mockType = mock(PlanType.class);
        MobilityHeader mockHeader = mock(MobilityHeader.class);
        when(mockType.getType()).thenReturn(PlanType.JOIN_PLATOON_AT_REAR);
        when(mockHeader.getSenderId()).thenReturn("1");
        when(unknownRequest.getPlanType()).thenReturn(mockType);
        when(unknownRequest.getHeader()).thenReturn(mockHeader);
        when(unknownRequest.getStrategyParams()).thenReturn("SIZE:1,SPEED:5.00,DTD:10.00");
        when(mockManager.getTotalPlatooningSize()).thenReturn(1);
        when(mockManager.getPlatoonRearDowntrackDistance()).thenReturn(50.0);
        mockPlugin.maxAllowedJoinGap = 50.0;
        mockPlugin.maxAllowedJoinTimeGap = 15.0;
        assertEquals(MobilityRequestResponse.ACK, leaderState.onMobilityRequestMessgae(unknownRequest));
        ArgumentCaptor<LeaderWaitingState> newState = ArgumentCaptor.forClass(LeaderWaitingState.class);
        verify(mockPlugin).setState(newState.capture());
        assertEquals("LeaderWaitingState", newState.getValue().toString());
    }
    
    @Test
    public void onJoinRequestOutOfMaxGapButInMaxTimeGap() {
        MobilityRequest unknownRequest = mock(MobilityRequest.class);
        PlanType mockType = mock(PlanType.class);
        MobilityHeader mockHeader = mock(MobilityHeader.class);
        when(mockType.getType()).thenReturn(PlanType.JOIN_PLATOON_AT_REAR);
        when(mockHeader.getSenderId()).thenReturn("1");
        when(unknownRequest.getPlanType()).thenReturn(mockType);
        when(unknownRequest.getHeader()).thenReturn(mockHeader);
        when(unknownRequest.getStrategyParams()).thenReturn("SIZE:1,SPEED:5.00,DTD:10.00");
        when(mockManager.getTotalPlatooningSize()).thenReturn(1);
        when(mockManager.getPlatoonRearDowntrackDistance()).thenReturn(80.0);
        mockPlugin.maxAllowedJoinGap = 50.0;
        mockPlugin.maxAllowedJoinTimeGap = 15.0;
        assertEquals(MobilityRequestResponse.ACK, leaderState.onMobilityRequestMessgae(unknownRequest));
        ArgumentCaptor<LeaderWaitingState> newState = ArgumentCaptor.forClass(LeaderWaitingState.class);
        verify(mockPlugin).setState(newState.capture());
        assertEquals("LeaderWaitingState", newState.getValue().toString());
    }
    
    @Test
    public void onJoinRequestOutOfMaxTimeGap() {
        MobilityRequest unknownRequest = mock(MobilityRequest.class);
        PlanType mockType = mock(PlanType.class);
        MobilityHeader mockHeader = mock(MobilityHeader.class);
        when(mockType.getType()).thenReturn(PlanType.JOIN_PLATOON_AT_REAR);
        when(mockHeader.getSenderId()).thenReturn("1");
        when(unknownRequest.getPlanType()).thenReturn(mockType);
        when(unknownRequest.getHeader()).thenReturn(mockHeader);
        when(unknownRequest.getStrategyParams()).thenReturn("SIZE:1,SPEED:5.00,DTD:10.00");
        when(mockManager.getTotalPlatooningSize()).thenReturn(1);
        when(mockManager.getPlatoonRearDowntrackDistance()).thenReturn(86.0);
        mockPlugin.maxAllowedJoinGap = 50.0;
        mockPlugin.maxAllowedJoinTimeGap = 15.0;
        assertEquals(MobilityRequestResponse.NACK, leaderState.onMobilityRequestMessgae(unknownRequest));
        verify(mockPlugin, times(0)).setState(any());
    }
    
    @Test
    public void onStatusOperationMessage() {
        MobilityOperation mockOperation = mock(MobilityOperation.class);
        when(mockOperation.getStrategyParams()).thenReturn(String.format(PlatooningPlugin.OPERATION_STATUS_PARAMS, 5.0, 20.0, 5.0));
        MobilityHeader header = mock(MobilityHeader.class);
        when(header.getSenderId()).thenReturn("1");
        when(header.getPlanId()).thenReturn("ABC");
        when(header.getSenderBsmId()).thenReturn("F");
        when(mockOperation.getHeader()).thenReturn(header);
        leaderState.onMobilityOperationMessage(mockOperation);
        ArgumentCaptor<String> senderId = ArgumentCaptor.forClass(String.class);
        ArgumentCaptor<String> platoonId = ArgumentCaptor.forClass(String.class);
        ArgumentCaptor<String> bsmId = ArgumentCaptor.forClass(String.class);
        ArgumentCaptor<String> params = ArgumentCaptor.forClass(String.class);
        verify(mockManager).memberUpdates(senderId.capture(), platoonId.capture(), bsmId.capture(), params.capture());
        assertEquals("1", senderId.getValue());
        assertEquals("ABC", platoonId.getValue());
        assertEquals("F", bsmId.getValue());
        assertEquals("CMDSPEED:5.00,DTD:20.00,SPEED:5.00", params.getValue());
    }
    
}
