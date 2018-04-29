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

import java.util.concurrent.atomic.AtomicBoolean;

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
        when(mockPlugin.getHandleMobilityPath()).thenReturn(new AtomicBoolean(true));
        candidateFollowerState = new CandidateFollowerState(mockPlugin, mockLog, pluginServiceLocator, "A", "E1B2");
    }
    
    // Remove current test cases because the logic in candidate follower state is the same as cruising plugin
    // TODO Add some test cases if necessary
}
