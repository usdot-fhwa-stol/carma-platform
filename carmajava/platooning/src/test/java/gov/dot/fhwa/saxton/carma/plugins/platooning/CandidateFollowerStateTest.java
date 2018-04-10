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
import static org.junit.Assert.assertTrue;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import org.junit.Before;
import org.junit.Test;
import org.mockito.ArgumentCaptor;

import gov.dot.fhwa.saxton.carma.guidance.ArbitratorService;
import gov.dot.fhwa.saxton.carma.guidance.IGuidanceCommands;
import gov.dot.fhwa.saxton.carma.guidance.ManeuverPlanner;
import gov.dot.fhwa.saxton.carma.guidance.arbitrator.TrajectoryPlanningResponse;
import gov.dot.fhwa.saxton.carma.guidance.conflictdetector.IConflictDetector;
import gov.dot.fhwa.saxton.carma.guidance.lightbar.ILightBarManager;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.AccStrategyManager;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.IManeuverInputs;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.NoOpAccStrategyFactory;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.SteadySpeed;
import gov.dot.fhwa.saxton.carma.guidance.mobilityrouter.IMobilityRouter;
import gov.dot.fhwa.saxton.carma.guidance.params.ParameterSource;
import gov.dot.fhwa.saxton.carma.guidance.plugins.PluginManagementService;
import gov.dot.fhwa.saxton.carma.guidance.plugins.PluginServiceLocator;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.IPubSubService;
import gov.dot.fhwa.saxton.carma.guidance.trajectory.Trajectory;
import gov.dot.fhwa.saxton.carma.guidance.util.ILogger;
import gov.dot.fhwa.saxton.carma.guidance.util.ILoggerFactory;
import gov.dot.fhwa.saxton.carma.guidance.util.LoggerManager;
import gov.dot.fhwa.saxton.carma.guidance.util.RouteService;
import gov.dot.fhwa.saxton.carma.guidance.util.SpeedLimit;
import gov.dot.fhwa.saxton.carma.guidance.util.trajectoryconverter.ITrajectoryConverter;

public class CandidateFollowerStateTest {
    
    protected PlatooningPlugin     mockPlugin;
    protected ILogger              mockLog;
    protected RouteService         mockRouteService;
    protected PlatoonManager       mockManager;
    protected IManeuverInputs      mockInputs;
    protected IMobilityRouter      mockRouter;
    protected ILoggerFactory       mockFact;
    protected PluginServiceLocator pluginServiceLocator;
    protected IPlatooningState     candidateFollowerState;
    protected ManeuverPlanner      planner;
    
    @Before
    public void setup() {
        mockFact             = mock(ILoggerFactory.class);
        mockPlugin           = mock(PlatooningPlugin.class);
        mockLog              = mock(ILogger.class);
        mockRouteService     = mock(RouteService.class);
        mockManager          = mock(PlatoonManager.class);
        mockInputs           = mock(IManeuverInputs.class);
        mockRouter           = mock(IMobilityRouter.class);
        planner              = new ManeuverPlanner(mock(IGuidanceCommands.class), mockInputs);
        pluginServiceLocator = new PluginServiceLocator(mock(ArbitratorService.class),    mock(PluginManagementService.class),
                                                        mock(IPubSubService.class),       mock(ParameterSource.class),
                                                        planner,                          mockRouteService,
                                                        mockRouter,                       mock(IConflictDetector.class),
                                                        mock(ITrajectoryConverter.class), mock(ILightBarManager.class));
        when(mockFact.createLoggerForClass(any())).thenReturn(mockLog);
        LoggerManager.setLoggerFactory(mockFact);
        NoOpAccStrategyFactory noOpAccStrategyFactory = new NoOpAccStrategyFactory();
        AccStrategyManager.setAccStrategyFactory(noOpAccStrategyFactory);
        when(mockPlugin.getPlatoonManager()).thenReturn(mockManager);
        when(mockPlugin.getManeuverInputs()).thenReturn(mockInputs);
        candidateFollowerState = new CandidateFollowerState(mockPlugin, mockLog, pluginServiceLocator, 5.0, "A", "E1B2");
    }
    
    @Test
    public void planTrajectoryToSpeedUp() {
        Trajectory traj = new Trajectory(0, 50);
        when(mockRouteService.isAlgorithmEnabledInRange(0.0, 50.0, mockPlugin.PLATOONING_FLAG)).thenReturn(true);
        when(mockInputs.getCurrentSpeed()).thenReturn(0.0);
        when(mockRouteService.getCurrentDowntrackDistance()).thenReturn(0.0);
        when(mockRouteService.getSpeedLimitAtLocation(0.0)).thenReturn(new SpeedLimit(100.0, 5.0));
        when(mockPlugin.getMaxAccel()).thenReturn(2.5);
        TrajectoryPlanningResponse tpr = candidateFollowerState.planTrajectory(traj, 0);
        assertTrue(tpr.getRequests().isEmpty());
        assertEquals(2, traj.getLongitudinalManeuvers().size());
        assertEquals(0.0, traj.getLongitudinalManeuvers().get(0).getStartSpeed(), 0.001);
        assertEquals(5.0, traj.getLongitudinalManeuvers().get(0).getTargetSpeed(), 0.001);
        assertEquals(0.0, traj.getLongitudinalManeuvers().get(0).getStartDistance(), 0.001);
        assertEquals(6.0, traj.getLongitudinalManeuvers().get(0).getEndDistance(), 0.001);
        assertEquals(5.0, traj.getLongitudinalManeuvers().get(1).getStartSpeed(), 0.001);
        assertEquals(5.0, traj.getLongitudinalManeuvers().get(1).getTargetSpeed(), 0.001);
        assertEquals(6.0, traj.getLongitudinalManeuvers().get(1).getStartDistance(), 0.001);
        assertEquals(31.0, traj.getLongitudinalManeuvers().get(1).getEndDistance(), 0.001);
    }
    
    @Test
    public void planTrajectoryToSpeedUpButWithoutEnoughSpace() {
        Trajectory traj = new Trajectory(0, 30);
        when(mockRouteService.isAlgorithmEnabledInRange(0.0, 30.0, mockPlugin.PLATOONING_FLAG)).thenReturn(true);
        when(mockInputs.getCurrentSpeed()).thenReturn(0.0);
        when(mockRouteService.getCurrentDowntrackDistance()).thenReturn(0.0);
        when(mockRouteService.getSpeedLimitAtLocation(0.0)).thenReturn(new SpeedLimit(100.0, 5.0));
        when(mockPlugin.getMaxAccel()).thenReturn(2.5);
        TrajectoryPlanningResponse tpr = candidateFollowerState.planTrajectory(traj, 0);
        assertEquals(1, tpr.getRequests().size());
        assertEquals(46.5, tpr.getProposedTrajectoryEnd().get(), 0.001);
    }
    
    @Test
    public void planTrajectoryToSpeedUpButWithNonEmptyTrajectory() {
        Trajectory traj = new Trajectory(0, 50);
        when(mockRouteService.isAlgorithmEnabledInRange(0.0, 50.0, mockPlugin.PLATOONING_FLAG)).thenReturn(true);
        when(mockInputs.getCurrentSpeed()).thenReturn(0.0);
        when(mockRouteService.getCurrentDowntrackDistance()).thenReturn(0.0);
        when(mockRouteService.getSpeedLimitAtLocation(0.0)).thenReturn(new SpeedLimit(100.0, 5.0));
        when(mockPlugin.getMaxAccel()).thenReturn(2.5);
        SteadySpeed mockManeuver = mock(SteadySpeed.class);
        when(mockManeuver.getStartDistance()).thenReturn(0.0);
        when(mockManeuver.getEndDistance()).thenReturn(40.0);
        when(mockManeuver.getStartSpeed()).thenReturn(3.0);
        when(mockManeuver.getTargetSpeed()).thenReturn(3.0);
        traj.addManeuver(mockManeuver);
        TrajectoryPlanningResponse tpr = candidateFollowerState.planTrajectory(traj, 0);
        assertEquals(1, tpr.getRequests().size());
        assertTrue(tpr.higherPriorityRequested());
    }
    
    @Test
    public void planedTrajectoryButInterrupted() {
        Trajectory traj = new Trajectory(0, 50);
        when(mockRouteService.isAlgorithmEnabledInRange(0.0, 50.0, mockPlugin.PLATOONING_FLAG)).thenReturn(true);
        when(mockInputs.getCurrentSpeed()).thenReturn(0.0);
        when(mockRouteService.getCurrentDowntrackDistance()).thenReturn(0.0);
        when(mockRouteService.getSpeedLimitAtLocation(0.0)).thenReturn(new SpeedLimit(100.0, 5.0));
        when(mockPlugin.getMaxAccel()).thenReturn(2.5);
        TrajectoryPlanningResponse tpr = candidateFollowerState.planTrajectory(traj, 0);
        Trajectory traj2 = new Trajectory(0, 60);
        when(mockRouteService.isAlgorithmEnabledInRange(0.0, 60.0, mockPlugin.PLATOONING_FLAG)).thenReturn(true);
        TrajectoryPlanningResponse tpr2 = candidateFollowerState.planTrajectory(traj2, 0);
        assertTrue(tpr2.getRequests().isEmpty());
        ArgumentCaptor<PlatoonLeaderState> newState = ArgumentCaptor.forClass(PlatoonLeaderState.class);
        verify(mockPlugin).setState(newState.capture());
        assertEquals("PlatoonLeaderState", newState.getValue().toString());
    }
    
}
