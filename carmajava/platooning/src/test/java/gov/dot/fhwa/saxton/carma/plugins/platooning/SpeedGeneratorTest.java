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
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

import org.junit.Before;
import org.junit.Test;

import gov.dot.fhwa.saxton.carma.guidance.ArbitratorService;
import gov.dot.fhwa.saxton.carma.guidance.IGuidanceCommands;
import gov.dot.fhwa.saxton.carma.guidance.ManeuverPlanner;
import gov.dot.fhwa.saxton.carma.guidance.conflictdetector.IConflictDetector;
import gov.dot.fhwa.saxton.carma.guidance.lightbar.ILightBarManager;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.IManeuverInputs;
import gov.dot.fhwa.saxton.carma.guidance.mobilityrouter.IMobilityRouter;
import gov.dot.fhwa.saxton.carma.guidance.params.ParameterSource;
import gov.dot.fhwa.saxton.carma.guidance.plugins.PluginManagementService;
import gov.dot.fhwa.saxton.carma.guidance.plugins.PluginServiceLocator;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.IPubSubService;
import gov.dot.fhwa.saxton.carma.guidance.util.ILogger;
import gov.dot.fhwa.saxton.carma.guidance.util.RouteService;
import gov.dot.fhwa.saxton.carma.guidance.util.trajectoryconverter.ITrajectoryConverter;

public class SpeedGeneratorTest {
    
    private PlatooningPlugin mockPlugin;
    private PlatoonManager   mockManager;
    private ILogger          mockLogger;
    private RouteService     mockRouteService;
    private IManeuverInputs  mockManeuverInputs;
    
    private PluginServiceLocator psl;
    private CommandGenerator     speedGenerator;
    
    
    @Before
    public void setup() {
        mockManager        = mock(PlatoonManager.class);
        mockLogger         = mock(ILogger.class);
        mockRouteService   = mock(RouteService.class);
        mockManeuverInputs = mock(IManeuverInputs.class);
        mockPlugin         = mock(PlatooningPlugin.class);
        psl = new PluginServiceLocator(mock(ArbitratorService.class), mock(PluginManagementService.class),
                                       mock(IPubSubService.class),    mock(ParameterSource.class),
                                       new ManeuverPlanner(mock(IGuidanceCommands.class), mockManeuverInputs),
                                       mockRouteService, mock(IMobilityRouter.class), mock(IConflictDetector.class),
                                       mock(ITrajectoryConverter.class), mock(ILightBarManager.class));
        when(mockPlugin.getTimeHeadway()).thenReturn(1.0);
        when(mockPlugin.getStandStillGap()).thenReturn(5.0);
        when(mockPlugin.getKpPID()).thenReturn(1.0);
        when(mockPlugin.getKiPID()).thenReturn(0.0);
        when(mockPlugin.getKdPID()).thenReturn(-0.5);
        when(mockPlugin.getPlatoonManager()).thenReturn(mockManager);
        when(mockPlugin.getCmdSpeedMaxAdjustment()).thenReturn(10.0);
    }
    
    @Test
    public void adjustSpeedAsTheFirstFollower() {
        // The host vehicle is the first follower and trying to maintain the gap with the leader vehicle
        PlatoonMember leader = new PlatoonMember("", 10.0, 10.0, 50, Long.MAX_VALUE);
        when(mockManager.getLeader()).thenReturn(leader);
        when(mockRouteService.getCurrentDowntrackDistance()).thenReturn(40.0);
        when(mockManeuverInputs.getCurrentSpeed()).thenReturn(10.0);
        when(mockManager.getPlatooningSize()).thenReturn(1);
        speedGenerator = new CommandGenerator(mockPlugin, mockLogger, psl);
        speedGenerator.generateSpeed(100.0);
        assertEquals(10.0, speedGenerator.getLastSpeedCommand(), 0.001);
        when(mockRouteService.getCurrentDowntrackDistance()).thenReturn(41.0);
        speedGenerator.generateSpeed(200.0);
        assertEquals(9.005, speedGenerator.getLastSpeedCommand(), 0.001);
        when(mockRouteService.getCurrentDowntrackDistance()).thenReturn(39.0);
        speedGenerator.generateSpeed(300.0);
        assertEquals(10.99, speedGenerator.getLastSpeedCommand(), 0.001);
    }
    
    @Test
    public void adjustSpeedAsTheSecondFollwer() {
        // The host vehicle is the second follower and trying to maintain the double time-gap with the leader vehicle
        PlatoonMember leader = new PlatoonMember("", 10.0, 10.0, 50.0, Long.MAX_VALUE);
        when(mockManager.getLeader()).thenReturn(leader);
        when(mockRouteService.getCurrentDowntrackDistance()).thenReturn(30.0);
        when(mockManeuverInputs.getCurrentSpeed()).thenReturn(10.0);
        when(mockManager.getPlatooningSize()).thenReturn(2);
        speedGenerator = new CommandGenerator(mockPlugin, mockLogger, psl);
        speedGenerator.generateSpeed(100.0);
        assertEquals(10.0, speedGenerator.getLastSpeedCommand(), 0.001);
        leader.vehiclePosition = 51.0;
        speedGenerator.generateSpeed(200.0);
        assertEquals(10.995, speedGenerator.getLastSpeedCommand(), 0.001);
        leader.vehiclePosition = 49.0;
        speedGenerator.generateSpeed(300.0);
        assertEquals(9.01, speedGenerator.getLastSpeedCommand(), 0.001);
    }
            
}
