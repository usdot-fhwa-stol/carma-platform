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

import gov.dot.fhwa.saxton.carma.guidance.maneuvers.IManeuverInputs;
import gov.dot.fhwa.saxton.carma.guidance.plugins.PluginServiceLocator;
import gov.dot.fhwa.saxton.carma.guidance.util.ILogger;

/**
 * This test is for both APF leader selection algorithm and pure leader following algorithm.
 */
public class LeaderSelectionTest {
    
    private PlatooningPlugin     mockPlugin;
    private ILogger              mockLogger;
    private IManeuverInputs      mockInputs;
    private PluginServiceLocator mockPSL;
    private PlatoonManager       manager;
    
    @Before
    public void setup() {
        mockLogger  = mock(ILogger.class);
        mockPlugin  = mock(PlatooningPlugin.class);
        mockPSL     = mock(PluginServiceLocator.class);
        mockInputs  = mock(IManeuverInputs.class);
        manager     = new PlatoonManager(mockPlugin, mockLogger, mockPSL);
        when(mockPlugin.getManeuverInputs()).thenReturn(mockInputs);
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
    public void pureLeaderFollowingInFollowerState() {
        manager.changeFromLeaderToFollower("3", "A");
        manager.platoon.add(new PlatoonMember("2", 5.0, 5.0, 30.0, System.currentTimeMillis()));
        manager.platoon.add(new PlatoonMember("3", 5.0, 5.0, 45.0, System.currentTimeMillis()));
        Collections.sort(manager.platoon, (a, b) -> (Double.compare(b.vehiclePosition, a.vehiclePosition)));
        PlatoonMember leader = manager.getLeader();
        assertEquals("3", leader.staticId);
        assertEquals(5.0, leader.commandSpeed, 0.01);
        assertEquals(5.0, leader.vehicleSpeed, 0.01);
        assertEquals(45.0, leader.vehiclePosition, 0.01);
    }
    
    @Test
    public void caseZeroInAPF() {
        // the second vehicle will always follow the first one
        when(mockPlugin.getAlgorithmType()).thenReturn(1);
        manager.changeFromLeaderToFollower("3", "A");
        manager.platoon.add(new PlatoonMember("3", 5.0, 5.0, 45.0, System.currentTimeMillis()));
        assertEquals("", manager.previousFunctionalLeaderID);
        assertEquals(-1, manager.previousFunctionalLeaderIndex);
        PlatoonMember leader = manager.getLeader();
        assertEquals("3", leader.staticId);
        assertEquals(5.0, leader.commandSpeed, 0.01);
        assertEquals(5.0, leader.vehicleSpeed, 0.01);
        assertEquals(45.0, leader.vehiclePosition, 0.01);
        assertEquals("3", manager.previousFunctionalLeaderID);
        assertEquals(0, manager.previousFunctionalLeaderIndex);
    }
    
    @Test
    public void caseOneInAPF() {
        // if it is the first time we run APF, return the first vehicle by default
        when(mockPlugin.getAlgorithmType()).thenReturn(1);
        manager.changeFromLeaderToFollower("3", "A");
        manager.platoon.add(new PlatoonMember("3", 5.0, 5.0, 45.0, System.currentTimeMillis()));
        manager.platoon.add(new PlatoonMember("2", 5.0, 5.0, 37.0, System.currentTimeMillis()));
        manager.platoon.add(new PlatoonMember("1", 5.0, 5.0, 30.0, System.currentTimeMillis()));
        Collections.sort(manager.platoon, (a, b) -> (Double.compare(b.vehiclePosition, a.vehiclePosition)));
        assertEquals("", manager.previousFunctionalLeaderID);
        assertEquals(-1, manager.previousFunctionalLeaderIndex);
        PlatoonMember leader = manager.getLeader();
        assertEquals("3", leader.staticId);
        assertEquals(5.0, leader.commandSpeed, 0.01);
        assertEquals(5.0, leader.vehicleSpeed, 0.01);
        assertEquals(45.0, leader.vehiclePosition, 0.01);
        assertEquals("3", manager.previousFunctionalLeaderID);
        assertEquals(0, manager.previousFunctionalLeaderIndex);
    }
    
    @Test
    public void caseTwoInAPF() {
        // assign to the preceding vehicle when the gap in front is very small
        when(mockInputs.getDistanceFromRouteStart()).thenReturn(10.0);
        when(mockInputs.getDistanceToFrontVehicle()).thenReturn(1.0);
        when(mockPlugin.getAlgorithmType()).thenReturn(1);
        manager.changeFromLeaderToFollower("3", "A");
        manager.platoon.add(new PlatoonMember("3", 5.0, 5.0, 45.0, System.currentTimeMillis()));
        manager.platoon.add(new PlatoonMember("2", 5.0, 5.0, 11.0, System.currentTimeMillis()));
        Collections.sort(manager.platoon, (a, b) -> (Double.compare(b.vehiclePosition, a.vehiclePosition)));
        manager.previousFunctionalLeaderID = "3";
        manager.previousFunctionalLeaderIndex = 0;
        PlatoonMember leader1 = manager.getLeader();
        assertEquals("2", leader1.staticId);
        assertEquals(5.0, leader1.commandSpeed, 0.01);
        assertEquals(5.0, leader1.vehicleSpeed, 0.01);
        assertEquals(11.0, leader1.vehiclePosition, 0.01);
        assertEquals("2", manager.previousFunctionalLeaderID);
        assertEquals(1, manager.previousFunctionalLeaderIndex);
        // assign to the preceding vehicle when the gap is still not large enough
        when(mockInputs.getDistanceToFrontVehicle()).thenReturn(1.99);
        manager.platoon.get(1).vehiclePosition = 12.0;
        PlatoonMember leader2 = manager.getLeader();
        assertEquals("2", leader2.staticId);
        assertEquals(5.0, leader2.commandSpeed, 0.01);
        assertEquals(5.0, leader2.vehicleSpeed, 0.01);
        assertEquals(12.0, leader2.vehiclePosition, 0.01);
        assertEquals("2", manager.previousFunctionalLeaderID);
        assertEquals(1, manager.previousFunctionalLeaderIndex);
    }
    
    @Test
    public void caseThreeInAPF1() {
        // if there is no violation in time headway, we should follow the first vehicle
        when(mockInputs.getDistanceFromRouteStart()).thenReturn(10.0);
        when(mockInputs.getDistanceToFrontVehicle()).thenReturn(19.5);
        when(mockInputs.getCurrentSpeed()).thenReturn(10.0);
        when(mockPlugin.getAlgorithmType()).thenReturn(1);
        manager.changeFromLeaderToFollower("3", "A");
        manager.platoon.add(new PlatoonMember("3", 10.0, 10.0, 68.5, System.currentTimeMillis()));
        manager.platoon.add(new PlatoonMember("2", 10.0, 10.0, 49, System.currentTimeMillis()));
        manager.platoon.add(new PlatoonMember("1", 10.0, 10.0, 29.5, System.currentTimeMillis()));
        Collections.sort(manager.platoon, (a, b) -> (Double.compare(b.vehiclePosition, a.vehiclePosition)));
        manager.previousFunctionalLeaderID = "3";
        manager.previousFunctionalLeaderIndex = 0;
        PlatoonMember leader = manager.getLeader();
        assertEquals("3", leader.staticId);
        assertEquals(10.0, leader.commandSpeed, 0.01);
        assertEquals(10.0, leader.vehicleSpeed, 0.01);
        assertEquals(68.5, leader.vehiclePosition, 0.01);
        assertEquals("3", manager.previousFunctionalLeaderID);
        assertEquals(0, manager.previousFunctionalLeaderIndex);
    }
    
    @Test
    public void caseThreeInAPF2() {
        // if there is a violation in time headway, we should follow the vehicle caused that violation
        when(mockInputs.getDistanceFromRouteStart()).thenReturn(10.0);
        when(mockInputs.getDistanceToFrontVehicle()).thenReturn(19.5);
        when(mockInputs.getCurrentSpeed()).thenReturn(10.0);
        when(mockPlugin.getAlgorithmType()).thenReturn(1);
        manager.changeFromLeaderToFollower("3", "A");
        // there is a violation on max_spacing between the second vehicle and the third vehicle in this platoon
        manager.platoon.add(new PlatoonMember("3", 10.0, 10.0, 71, System.currentTimeMillis()));
        manager.platoon.add(new PlatoonMember("2", 10.0, 10.0, 51.5, System.currentTimeMillis()));
        manager.platoon.add(new PlatoonMember("1", 10.0, 10.0, 29.5, System.currentTimeMillis()));
        Collections.sort(manager.platoon, (a, b) -> (Double.compare(b.vehiclePosition, a.vehiclePosition)));
        manager.previousFunctionalLeaderID = "3";
        manager.previousFunctionalLeaderIndex = 0;
        PlatoonMember leader = manager.getLeader();
        assertEquals("1", leader.staticId);
        assertEquals(10.0, leader.commandSpeed, 0.01);
        assertEquals(10.0, leader.vehicleSpeed, 0.01);
        assertEquals(29.5, leader.vehiclePosition, 0.01);
        assertEquals("1", manager.previousFunctionalLeaderID);
        assertEquals(2, manager.previousFunctionalLeaderIndex);
    }
    
    @Test
    public void caseFourInAPF() {
        // not following the first vehicle in previous timesetp but change back to the first one when no more violation
        when(mockInputs.getDistanceFromRouteStart()).thenReturn(10.0);
        when(mockInputs.getDistanceToFrontVehicle()).thenReturn(18.0);
        when(mockInputs.getCurrentSpeed()).thenReturn(10.0);
        when(mockPlugin.getAlgorithmType()).thenReturn(1);
        manager.changeFromLeaderToFollower("3", "A");
        // there is a violation on max_spacing between the second vehicle and the third vehicle in this platoon
        manager.platoon.add(new PlatoonMember("3", 10.0, 10.0, 64, System.currentTimeMillis()));
        manager.platoon.add(new PlatoonMember("2", 10.0, 10.0, 46, System.currentTimeMillis()));
        manager.platoon.add(new PlatoonMember("1", 10.0, 10.0, 28, System.currentTimeMillis()));
        Collections.sort(manager.platoon, (a, b) -> (Double.compare(b.vehiclePosition, a.vehiclePosition)));
        manager.previousFunctionalLeaderID = "2";
        manager.previousFunctionalLeaderIndex = 1;
        PlatoonMember leader = manager.getLeader();
        assertEquals("3", leader.staticId);
        assertEquals(10.0, leader.commandSpeed, 0.01);
        assertEquals(10.0, leader.vehicleSpeed, 0.01);
        assertEquals(64.0, leader.vehiclePosition, 0.01);
        assertEquals("3", manager.previousFunctionalLeaderID);
        assertEquals(0, manager.previousFunctionalLeaderIndex);
    }
    
    @Test
    public void caseFiveInAPF() {
        // not following the first vehicle in previous timesetp and cannot change in this timestep
        when(mockInputs.getDistanceFromRouteStart()).thenReturn(10.0);
        when(mockInputs.getDistanceToFrontVehicle()).thenReturn(18.0);
        when(mockInputs.getCurrentSpeed()).thenReturn(10.0);
        when(mockPlugin.getAlgorithmType()).thenReturn(1);
        manager.changeFromLeaderToFollower("3", "A");
        // there is a violation on max_spacing between the second vehicle and the third vehicle in this platoon
        manager.platoon.add(new PlatoonMember("3", 10.0, 10.0, 63, System.currentTimeMillis()));
        manager.platoon.add(new PlatoonMember("2", 10.0, 10.0, 45, System.currentTimeMillis()));
        manager.platoon.add(new PlatoonMember("1", 10.0, 10.0, 28, System.currentTimeMillis()));
        Collections.sort(manager.platoon, (a, b) -> (Double.compare(b.vehiclePosition, a.vehiclePosition)));
        manager.previousFunctionalLeaderID = "2";
        manager.previousFunctionalLeaderIndex = 1;
        PlatoonMember leader = manager.getLeader();
        assertEquals("2", leader.staticId);
    }
}
