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
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.IManeuverInputs;
import gov.dot.fhwa.saxton.carma.guidance.params.ParameterSource;
import gov.dot.fhwa.saxton.carma.guidance.plugins.PluginManagementService;
import gov.dot.fhwa.saxton.carma.guidance.plugins.PluginServiceLocator;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.IPubSubService;
import gov.dot.fhwa.saxton.carma.guidance.util.GuidanceRouteService;
import gov.dot.fhwa.saxton.carma.guidance.util.ILogger;

public class SpeedGenerationTest {
    
    private PlatooningPlugin plugin;
    private PlatoonManager mockManager;
    private ILogger mockLogger;
    private PluginServiceLocator psl;
    private CommandGenerator speedGenerator;
    private GuidanceRouteService mockRouteService;
    private IManeuverInputs mockManeuverInputs;
    
    @Before
    public void setup() {
        mockManager = mock(PlatoonManager.class);
        mockLogger = mock(ILogger.class);
        mockRouteService = mock(GuidanceRouteService.class);
        mockManeuverInputs = mock(IManeuverInputs.class);
        psl = new PluginServiceLocator(mock(ArbitratorService.class),
                mock(PluginManagementService.class), mock(IPubSubService.class), mock(ParameterSource.class),
                new ManeuverPlanner(mock(IGuidanceCommands.class), mockManeuverInputs), mockRouteService);
        plugin = mock(PlatooningPlugin.class);
        when(plugin.getTimeHeadway()).thenReturn(1.0);
        when(plugin.getStandStillGap()).thenReturn(5.0);
        when(plugin.getKpPID()).thenReturn(1.0);
        when(plugin.getKiPID()).thenReturn(0.0);
        when(plugin.getKdPID()).thenReturn(0.5);
    }
    
    @Test
    public void adjustSpeedTest() {
        // In platoon with two vehicles, the rear vehicle is trying to maintain the gap with the front vehicle
        when(plugin.getPlatoonManager()).thenReturn(mockManager);
        PlatoonMember member = new PlatoonMember("", 10.0, 10.0, 50, Long.MAX_VALUE);
        when(mockManager.getLeader()).thenReturn(member);
        when(mockRouteService.getCurrentDowntrackDistance()).thenReturn(40.0);
        when(mockManeuverInputs.getCurrentSpeed()).thenReturn(9.0);
        when(mockManager.getPlatooningSize()).thenReturn(1);
        speedGenerator = new CommandGenerator(plugin, mockLogger, psl);
        speedGenerator.generateSpeed(100.0);
        assertEquals(9.0, speedGenerator.getLastSpeedCommand(), 0.001);
        when(mockManeuverInputs.getCurrentSpeed()).thenReturn(11.0);
        speedGenerator.generateSpeed(200.0);
        assertEquals(11.01, speedGenerator.getLastSpeedCommand(), 0.001);
        when(mockManeuverInputs.getCurrentSpeed()).thenReturn(10.0);
        speedGenerator.generateSpeed(300.0);
        assertEquals(9.995, speedGenerator.getLastSpeedCommand(), 0.001);
    }
    
}
