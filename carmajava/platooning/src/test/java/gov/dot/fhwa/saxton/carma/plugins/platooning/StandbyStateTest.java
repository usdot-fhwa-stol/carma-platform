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

import cav_msgs.MobilityRequest;
import gov.dot.fhwa.saxton.carma.guidance.ArbitratorService;
import gov.dot.fhwa.saxton.carma.guidance.ManeuverPlanner;
import gov.dot.fhwa.saxton.carma.guidance.TrackingService;
import gov.dot.fhwa.saxton.carma.guidance.arbitrator.TrajectoryPlanningResponse;
import gov.dot.fhwa.saxton.carma.guidance.conflictdetector.IConflictDetector;
import gov.dot.fhwa.saxton.carma.guidance.lightbar.ILightBarManager;
import gov.dot.fhwa.saxton.carma.guidance.mobilityrouter.IMobilityRouter;
import gov.dot.fhwa.saxton.carma.guidance.mobilityrouter.MobilityRequestResponse;
import gov.dot.fhwa.saxton.carma.guidance.params.ParameterSource;
import gov.dot.fhwa.saxton.carma.guidance.plugins.PluginManagementService;
import gov.dot.fhwa.saxton.carma.guidance.plugins.PluginServiceLocator;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.IPubSubService;
import gov.dot.fhwa.saxton.carma.guidance.trajectory.Trajectory;
import gov.dot.fhwa.saxton.carma.guidance.util.ILogger;
import gov.dot.fhwa.saxton.carma.guidance.util.RouteService;
import gov.dot.fhwa.saxton.carma.guidance.util.trajectoryconverter.ITrajectoryConverter;

// This test only focus on the behavior of IPlatooningState API.
public class StandbyStateTest {
    
    protected PlatooningPlugin     mockPlugin;
    protected ILogger              mockLog;
    protected RouteService         mockRouteService;
    protected PlatoonManager       mockManager;
    protected PluginServiceLocator pluginServiceLocator;
    protected IPlatooningState     standbyState;
    
    @Before
    public void setup() {
        mockPlugin           = mock(PlatooningPlugin.class);
        mockLog              = mock(ILogger.class);
        mockRouteService     = mock(RouteService.class);
        mockManager          = mock(PlatoonManager.class);
        pluginServiceLocator = new PluginServiceLocator(mock(ArbitratorService.class),    mock(PluginManagementService.class),
                                                        mock(IPubSubService.class),       mock(ParameterSource.class),
                                                        mock(ManeuverPlanner.class),      mockRouteService,
                                                        mock(IMobilityRouter.class),      mock(IConflictDetector.class),
                                                        mock(ITrajectoryConverter.class), mock(ILightBarManager.class),
                                                        mock(TrackingService.class));
        when(mockPlugin.getHandleMobilityPath()).thenReturn(new AtomicBoolean(true));
        when(mockPlugin.getPlatoonManager()).thenReturn(mockManager);
        standbyState         = new StandbyState(mockPlugin, mockLog, pluginServiceLocator);
    }
    
    @Test
    public void planTrajectoryWithoutAnyWindow() {
        Trajectory traj = new Trajectory(0, 50);
        when(mockRouteService.isAlgorithmEnabledInRange(0.0, 50.0, mockPlugin.PLATOONING_FLAG)).thenReturn(false);
        TrajectoryPlanningResponse tpr = standbyState.planTrajectory(traj, 0);
        assertTrue(tpr.getRequests().isEmpty());
        assertNull(traj.getComplexManeuver());
        assertTrue(traj.getLongitudinalManeuvers().isEmpty());
        assertTrue(traj.getLateralManeuvers().isEmpty());
        verify(mockPlugin, times(0)).setState(any());
    }
    
    @Test
    public void planTrajectoryWithAnOpenWindow() {
        Trajectory traj = new Trajectory(0, 50);
        when(mockRouteService.isAlgorithmEnabledInRange(0.0, 50.0, mockPlugin.PLATOONING_FLAG)).thenReturn(true);
        TrajectoryPlanningResponse tpr = standbyState.planTrajectory(traj, 0);
        assertNull(traj.getComplexManeuver());
        assertTrue(traj.getLongitudinalManeuvers().isEmpty());
        assertTrue(traj.getLateralManeuvers().isEmpty());
        assertTrue(!tpr.getRequests().isEmpty());
        assertEquals(50, tpr.getProposedReplanDelay().get().longValue());
        ArgumentCaptor<LeaderState> newState = ArgumentCaptor.forClass(LeaderState.class);
        verify(mockPlugin).setState(newState.capture());
        assertEquals("PlatoonLeaderState", newState.getValue().toString());
    }
    
    @Test
    public void noResponseOnMobilityRequestMessgae() {
        MobilityRequest mockRequest = mock(MobilityRequest.class);
        assertEquals(MobilityRequestResponse.NO_RESPONSE, standbyState.onMobilityRequestMessgae(mockRequest));
    }
    
}
