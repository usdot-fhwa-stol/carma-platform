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

import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicBoolean;

import org.junit.Before;
import org.junit.Test;
import org.mockito.ArgumentCaptor;

import cav_msgs.LocationECEF;
import cav_msgs.MobilityHeader;
import cav_msgs.MobilityOperation;
import cav_msgs.MobilityRequest;
import cav_msgs.MobilityResponse;
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
import gov.dot.fhwa.saxton.carma.guidance.util.SpeedLimit;
import gov.dot.fhwa.saxton.carma.guidance.util.trajectoryconverter.ITrajectoryConverter;

// This test only focus on the behavior of IPlatooningState API.
public class LeaderStateTest {

    protected PlatooningPlugin     mockPlugin;
    protected ILogger              mockLog;
    protected RouteService         mockRouteService;
    protected PlatoonManager       mockManager;
    protected IManeuverInputs      mockInputs;
    protected IMobilityRouter      mockRouter;
    protected PluginServiceLocator pluginServiceLocator;
    protected IPlatooningState     leaderState;
    
    @Before
    public void setup() {
        mockPlugin           = mock(PlatooningPlugin.class);
        mockLog              = mock(ILogger.class);
        mockRouteService     = mock(RouteService.class);
        mockManager          = mock(PlatoonManager.class);
        mockInputs           = mock(IManeuverInputs.class);
        mockRouter           = mock(IMobilityRouter.class);
        pluginServiceLocator = new PluginServiceLocator(mock(ArbitratorService.class),    mock(PluginManagementService.class),
                                                        mock(IPubSubService.class),       mock(ParameterSource.class),
                                                        mock(ManeuverPlanner.class),      mockRouteService,
                                                        mockRouter,                       mock(IConflictDetector.class),
                                                        mock(ITrajectoryConverter.class), mock(ILightBarManager.class),
                                                        mock(TrackingService.class));
        when(mockPlugin.getPlatoonManager()).thenReturn(mockManager);
        when(mockPlugin.getMaxPlatoonSize()).thenReturn(5);
        when(mockPlugin.getManeuverInputs()).thenReturn(mockInputs);
        when(mockPlugin.getHandleMobilityPath()).thenReturn(new AtomicBoolean(true));
        leaderState = new LeaderState(mockPlugin, mockLog, pluginServiceLocator);
    }
    
    @Test
    public void planTrajectoryWithoutAnyWindow() {
        Trajectory traj = new Trajectory(0, 50);
        when(mockRouteService.isAlgorithmEnabledInRange(0.0, 50.0, mockPlugin.PLATOONING_FLAG)).thenReturn(false);
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
        when(mockRouteService.isAlgorithmEnabledInRange(0.0, 50.0, mockPlugin.PLATOONING_FLAG)).thenReturn(true);
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
        when(unknownRequest.getStrategyParams()).thenReturn("SIZE:1,ACCEL:2.5,DTD:10.5");
        when(mockManager.getPlatooningSize()).thenReturn(4);
        assertEquals(MobilityRequestResponse.NACK, leaderState.onMobilityRequestMessgae(unknownRequest));
    }
    
    @Test
    public void onJoinRequestWithDifferentGaps() {
        MobilityRequest unknownRequest = mock(MobilityRequest.class);
        PlanType mockType = mock(PlanType.class);
        MobilityHeader mockHeader = mock(MobilityHeader.class);
        when(mockType.getType()).thenReturn(PlanType.JOIN_PLATOON_AT_REAR);
        when(mockHeader.getSenderId()).thenReturn("1");
        when(unknownRequest.getPlanType()).thenReturn(mockType);
        when(unknownRequest.getHeader()).thenReturn(mockHeader);
        when(unknownRequest.getStrategyParams()).thenReturn("SIZE:1,ACCEL:2.5,DTD:10");
        when(mockManager.getPlatooningSize()).thenReturn(0);
        when(mockManager.getPlatoonRearDowntrackDistance()).thenReturn(50.0);
        when(mockPlugin.getDesiredJoinTimeGap()).thenReturn(4.0);
        when(mockInputs.getCurrentSpeed()).thenReturn(5.0);
        when(mockRouteService.getSpeedLimitAtLocation(50.0)).thenReturn(new SpeedLimit(55, 10));
        when(mockInputs.getResponseLag()).thenReturn(1.0);
        when(mockPlugin.getMaxJoinTime()).thenReturn(6.0);
        assertEquals(MobilityRequestResponse.NACK, leaderState.onMobilityRequestMessgae(unknownRequest));
        verify(mockPlugin, times(0)).setState(any());
        when(mockPlugin.getMaxJoinTime()).thenReturn(10.0);
        assertEquals(MobilityRequestResponse.ACK, leaderState.onMobilityRequestMessgae(unknownRequest));
        ArgumentCaptor<LeaderWaitingState> newState = ArgumentCaptor.forClass(LeaderWaitingState.class);
        verify(mockPlugin).setState(newState.capture());
        assertEquals("LeaderWaitingState", newState.getValue().toString());
    }
    
    @Test
    public void onStatusOperationMessage() {
        MobilityOperation mockOperation = mock(MobilityOperation.class);
        when(mockOperation.getStrategyParams()).thenReturn(String.format(mockPlugin.OPERATION_STATUS_PARAMS, 5.0, 20.0, 5.0));
        MobilityHeader header = mock(MobilityHeader.class);
        when(header.getSenderId()).thenReturn("1");
        when(header.getPlanId()).thenReturn("ABC");
        when(mockOperation.getHeader()).thenReturn(header);
        leaderState.onMobilityOperationMessage(mockOperation);
        ArgumentCaptor<String> senderId = ArgumentCaptor.forClass(String.class);
        ArgumentCaptor<String> platoonId = ArgumentCaptor.forClass(String.class);
        ArgumentCaptor<String> params = ArgumentCaptor.forClass(String.class);
        verify(mockManager).memberUpdates(senderId.capture(), platoonId.capture(), params.capture());
        assertEquals("1", senderId.getValue());
        assertEquals("ABC", platoonId.getValue());
        assertEquals("CMDSPEED:5.00,DTD:20.00,SPEED:5.00", params.getValue());
    }
    
    @Test
    public void onInfoOperationMessageBehindTheHostVehicle() {
        MobilityOperation mockOperation = mock(MobilityOperation.class);
        MobilityHeader header = mock(MobilityHeader.class);
        when(header.getPlanId()).thenReturn("ABC");
        when(mockOperation.getHeader()).thenReturn(header);
        when(mockOperation.getStrategyParams()).thenReturn(String.format(mockPlugin.OPERATION_INFO_PARAMS_F, "00000000", 5.0, 5.0, 1));
        when(mockRouteService.getCurrentDowntrackDistance()).thenReturn(30.0);
        leaderState.onMobilityOperationMessage(mockOperation);
        verify(mockPlugin, times(0)).getMobilityRequestPublisher();
    }
    
    @Test
    public void onInfoOperationMessageInFrontOfHostVehicleAndACK() {
        MobilityOperation mockOperation = mock(MobilityOperation.class);
        MobilityHeader header = mock(MobilityHeader.class);
        when(header.getPlanId()).thenReturn("ABC");
        when(mockOperation.getHeader()).thenReturn(header);
        when(mockOperation.getStrategyParams()).thenReturn(String.format(mockPlugin.OPERATION_INFO_PARAMS_F, "00000000", 5.0, 5.0, 1));
        when(mockRouteService.getCurrentDowntrackDistance()).thenReturn(30.0);
        when(mockPlugin.getDesiredJoinTimeGap()).thenReturn(4.0);
        when(mockRouteService.getSpeedLimitAtLocation(30.0)).thenReturn(new SpeedLimit(30, 10.0));
        IPublisher<MobilityRequest> mockPublisher = mock(IPublisher.class);
        MobilityRequest mockRequest = mock(MobilityRequest.class);
        MobilityHeader requestHeader = (mock(MobilityHeader.class));
        LocationECEF mockLocation = mock(LocationECEF.class);
        PlanType mockType = mock(PlanType.class);
        when(mockPlugin.getMobilityRequestPublisher()).thenReturn(mockPublisher);
        when(mockRequest.getHeader()).thenReturn(requestHeader);
        when(mockRequest.getLocation()).thenReturn(mockLocation);
        when(mockRequest.getPlanType()).thenReturn(mockType);
        when(mockPublisher.newMessage()).thenReturn(mockRequest);
        when(mockRouter.getHostMobilityId()).thenReturn("B");
        when(mockManager.getPlatooningSize()).thenReturn(2);
        when(mockPlugin.getMaxAccel()).thenReturn(2.5);
        leaderState.onMobilityOperationMessage(mockOperation);
        ArgumentCaptor<String> params = ArgumentCaptor.forClass(String.class);
        ArgumentCaptor<String> targetId = ArgumentCaptor.forClass(String.class);
        ArgumentCaptor<String> planId = ArgumentCaptor.forClass(String.class);
        verify(mockRequest).setStrategyParams(params.capture());
        verify(requestHeader).setRecipientId(targetId.capture());
        verify(requestHeader).setPlanId(planId.capture());
        verify(mockPublisher).publish(any());
        assertEquals("SIZE:3,MAX_ACCEL:2.50,DTD:30.00", params.getValue());
        assertEquals("A", targetId.getValue());
        MobilityResponse response = mock(MobilityResponse.class);
        MobilityHeader responseHeader = mock(MobilityHeader.class);
        when(response.getHeader()).thenReturn(responseHeader);
        when(responseHeader.getPlanId()).thenReturn(planId.getValue());
        when(responseHeader.getSenderId()).thenReturn("A");
        when(response.getIsAccepted()).thenReturn(true);
        leaderState.onMobilityResponseMessage(response);
        ArgumentCaptor<CandidateFollowerState> newState = ArgumentCaptor.forClass(CandidateFollowerState.class);
        verify(mockPlugin).setState(newState.capture());
        assertEquals("CandidateFollowerState", newState.getValue().toString());
    }
    
}
