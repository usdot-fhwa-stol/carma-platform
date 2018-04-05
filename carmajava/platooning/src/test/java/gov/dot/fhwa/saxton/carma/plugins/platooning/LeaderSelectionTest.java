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
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

import java.util.Collections;

import org.junit.Before;
import org.junit.Test;

import gov.dot.fhwa.saxton.carma.guidance.plugins.PluginServiceLocator;
import gov.dot.fhwa.saxton.carma.guidance.util.ILogger;

/**
 * This test is only for non-APF leader selection algorithm.
 * TODO More test cases should be added in order to test APF leader selection.
 */
public class LeaderSelectionTest {
    
    private PlatooningPlugin     mockPlugin;
    private ILogger              mockLogger;
    private PluginServiceLocator mockPSL;
    private PlatoonManager       manager;
    
    @Before
    public void setup() {
        mockLogger  = mock(ILogger.class);
        mockPlugin  = mock(PlatooningPlugin.class);
        mockPSL     = mock(PluginServiceLocator.class);
        manager     = new PlatoonManager(mockPlugin, mockLogger, mockPSL);
        // Disable APF algorithm by default
        when(mockPlugin.getAlgorithmType()).thenReturn(0);
        // The following parameters are for APF leader selection algorithm
        when(mockPlugin.getLowerBoundary()).thenReturn(1.65);
        when(mockPlugin.getUpperBoundary()).thenReturn(1.75);
        when(mockPlugin.getMaxSpacing()).thenReturn(2.0);
        when(mockPlugin.getMinSpacing()).thenReturn(1.9);
        when(mockPlugin.getMinGap()).thenReturn(12.0);
        when(mockPlugin.getMaxGap()).thenReturn(14.0);
    }
    
    @Test
    public void returnNullOnGetLeaderInLeaderState() {
        PlatoonMember nullLeader = manager.getLeader();
        assertNull(nullLeader);
    }
    
    @Test
    public void returnTheFirstVehicleWhenAPFDisabledInFollowerState() {
        manager.changeFromLeaderToFollower("3", "A");
        manager.platoon.add(new PlatoonMember("1", 5.0, 5.0, 50.0, System.currentTimeMillis()));
        manager.platoon.add(new PlatoonMember("2", 5.0, 5.0, 60.0, System.currentTimeMillis()));
        manager.platoon.add(new PlatoonMember("3", 5.0, 5.0, 45.0, System.currentTimeMillis()));
        Collections.sort(manager.platoon, (a, b) -> (Double.compare(b.vehiclePosition, a.vehiclePosition)));
        PlatoonMember leader = manager.getLeader();
        assertEquals("2", leader.staticId);
        assertEquals(5.0, leader.commandSpeed, 0.01);
        assertEquals(5.0, leader.vehicleSpeed, 0.01);
        assertEquals(60.0, leader.vehiclePosition, 0.01);
    }
}
