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
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

import org.junit.Before;
import org.junit.Test;

import gov.dot.fhwa.saxton.carma.guidance.ArbitratorService;
import gov.dot.fhwa.saxton.carma.guidance.IGuidanceCommands;
import gov.dot.fhwa.saxton.carma.guidance.ManeuverPlanner;
import gov.dot.fhwa.saxton.carma.guidance.arbitrator.TrajectoryPlanningResponse;
import gov.dot.fhwa.saxton.carma.guidance.conflictdetector.IConflictDetector;
import gov.dot.fhwa.saxton.carma.guidance.lightbar.ILightBarManager;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.AccStrategyManager;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.IManeuverInputs;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.NoOpAccStrategyFactory;
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
import gov.dot.fhwa.saxton.carma.guidance.util.trajectoryconverter.ITrajectoryConverter;

public class FollowerStateTest {
    
    protected PlatooningPlugin     mockPlugin;
    protected ILogger              mockLog;
    protected RouteService         mockRouteService;
    protected PlatoonManager       mockManager;
    protected IManeuverInputs      mockInputs;
    protected ILoggerFactory       mockFact;
    protected CommandGenerator     mockCmdGenerator;
    protected PluginServiceLocator pluginServiceLocator;
    protected IPlatooningState     followerState;
    protected ManeuverPlanner      planner;
    
    @Before
    public void setup() {
        mockFact             = mock(ILoggerFactory.class);
        mockPlugin           = mock(PlatooningPlugin.class);
        mockLog              = mock(ILogger.class);
        mockRouteService     = mock(RouteService.class);
        mockManager          = mock(PlatoonManager.class);
        mockInputs           = mock(IManeuverInputs.class);
        mockCmdGenerator     = mock(CommandGenerator.class);
        planner              = new ManeuverPlanner(mock(IGuidanceCommands.class), mockInputs);
        pluginServiceLocator = new PluginServiceLocator(mock(ArbitratorService.class),    mock(PluginManagementService.class),
                                                        mock(IPubSubService.class),       mock(ParameterSource.class),
                                                        planner,                          mockRouteService,
                                                        mock(IMobilityRouter.class),      mock(IConflictDetector.class),
                                                        mock(ITrajectoryConverter.class), mock(ILightBarManager.class));
        when(mockFact.createLoggerForClass(any())).thenReturn(mockLog);
        LoggerManager.setLoggerFactory(mockFact);
        NoOpAccStrategyFactory noOpAccStrategyFactory = new NoOpAccStrategyFactory();
        AccStrategyManager.setAccStrategyFactory(noOpAccStrategyFactory);
        when(mockPlugin.getPlatoonManager()).thenReturn(mockManager);
        when(mockPlugin.getManeuverInputs()).thenReturn(mockInputs);
        when(mockPlugin.getCommandGenerator()).thenReturn(mockCmdGenerator);
        followerState = new FollowerState(mockPlugin, mockLog, pluginServiceLocator);
    }
    
    @Test
    public void planComplexManeuver() {
        Trajectory traj = new Trajectory(0, 50.0);
        when(mockRouteService.isAlgorithmEnabledInRange(0.0, 50.0, mockPlugin.PLATOONING_FLAG)).thenReturn(true);
        when(mockRouteService.getAlgorithmEnabledWindowInRange(0, 50.0, mockPlugin.PLATOONING_FLAG)).thenReturn(new double[]{0.0, 50.0});
        when(mockPlugin.getMinimumManeuverLength()).thenReturn(15.0);
        TrajectoryPlanningResponse tpr = followerState.planTrajectory(traj, 0);
        assertEquals(0, tpr.getRequests().size());
        assertEquals(0.0, traj.getComplexManeuver().getStartDistance(), 0.001);
        assertEquals(50.0, traj.getComplexManeuver().getEndDistance(), 0.001);
    }
}
