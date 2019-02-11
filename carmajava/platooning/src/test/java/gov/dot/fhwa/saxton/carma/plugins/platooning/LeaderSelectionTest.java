/*
 * Copyright (C) 2018-2019 LEIDOS.
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

import java.time.Clock;

import org.junit.Before;
import org.junit.Test;

import gov.dot.fhwa.saxton.carma.guidance.maneuvers.IManeuverInputs;
import gov.dot.fhwa.saxton.carma.guidance.mobilityrouter.IMobilityRouter;
import gov.dot.fhwa.saxton.carma.guidance.plugins.PluginServiceLocator;
import gov.dot.fhwa.saxton.carma.guidance.util.ILogger;
import gov.dot.fhwa.saxton.carma.guidance.util.RouteService;

/**
 * This test is for both APF leader selection algorithm and pure leader following algorithm.
 */
public class LeaderSelectionTest {
    
    private PlatooningPlugin     mockPlugin;
    private ILogger              mockLogger;
    private IManeuverInputs      mockInputs;
    private PluginServiceLocator mockPSL;
    private PlatoonManager       manager;
    private IMobilityRouter      mockRouter;
    private RouteService         mockRouteService;
    
    @Before
    public void setup() {
        mockLogger  = mock(ILogger.class);
        mockPlugin  = mock(PlatooningPlugin.class);
        mockPSL     = mock(PluginServiceLocator.class);
        mockInputs  = mock(IManeuverInputs.class);
        mockRouter  = mock(IMobilityRouter.class);
        mockRouteService = mock(RouteService.class);
        when(mockPSL.getMobilityRouter()).thenReturn(mockRouter);
        when(mockPSL.getRouteService()).thenReturn(mockRouteService);
        when(mockRouter.getHostMobilityId()).thenReturn("FFFFFFFF");
        manager     = new PlatoonManager(mockPlugin, mockLogger, mockPSL, Clock.systemUTC());
        when(mockPlugin.getManeuverInputs()).thenReturn(mockInputs);
        // Disable APF algorithm by default
        mockPlugin.algorithmType = 0;
        // The following parameters are for APF leader selection algorithm
        mockPlugin.lowerBoundary = 1.65;
        mockPlugin.upperBoundary = 1.75;
        mockPlugin.maxSpacing = 2.0;
        mockPlugin.minSpacing = 1.9;
        mockPlugin.minGap = 12.0;
        mockPlugin.maxGap = 14.0;
    }
    
    @Test
    public void returnNullOnGetLeaderInLeaderState() {
        PlatoonMember nullLeader = manager.getLeader();
        assertNull(nullLeader);
    }
    
    @Test
    public void pureLeaderFollowingInFollowerState() {
        manager.changeFromLeaderToFollower("A");
        when(mockRouteService.getCurrentDowntrackDistance()).thenReturn(35.0);
        manager.memberUpdates("BLACK", "A", "00000000", "CMDSPEED:1.00,DTD:50.00,SPEED:1.00");
        manager.memberUpdates("WHITE", "A", "00000000", "CMDSPEED:1.00,DTD:40.00,SPEED:1.00");
        manager.memberUpdates("GREY", "A", "00000000", "CMDSPEED:1.00,DTD:60.00,SPEED:1.00");
        PlatoonMember leader = manager.getLeader();
        assertEquals("GREY", leader.staticId);
        assertEquals(1.0, leader.commandSpeed, 0.01);
        assertEquals(1.0, leader.vehicleSpeed, 0.01);
        assertEquals(60.0, leader.vehiclePosition, 0.01);
    }
    
    @Test
    public void caseZeroInAPF() {
        // the second vehicle will always follow the first one
        mockPlugin.algorithmType = 1;
        manager.changeFromLeaderToFollower("A");
        when(mockRouteService.getCurrentDowntrackDistance()).thenReturn(35.0);
        manager.memberUpdates("BLACK", "A", "00000000", "CMDSPEED:1.00,DTD:50.00,SPEED:1.00");
        PlatoonMember leader = manager.getLeader();
        assertEquals("BLACK", leader.staticId);
        assertEquals(1.0, leader.commandSpeed, 0.01);
        assertEquals(1.0, leader.vehicleSpeed, 0.01);
        assertEquals(50.0, leader.vehiclePosition, 0.01);
    }
    
    @Test
    public void caseOneInAPF() {
        // if it is the first time we run APF, return the first vehicle by default
        mockPlugin.algorithmType = 1;
        manager.changeFromLeaderToFollower("A");
        when(mockRouteService.getCurrentDowntrackDistance()).thenReturn(35.0);
        manager.memberUpdates("3", "A", "00000000", "CMDSPEED:5.00,DTD:45.00,SPEED:5.00");
        manager.memberUpdates("2", "A", "00000000", "CMDSPEED:5.00,DTD:37.00,SPEED:5.00");
        manager.memberUpdates("1", "A", "00000000", "CMDSPEED:5.00,DTD:30.00,SPEED:5.00");
        PlatoonMember leader = manager.getLeader();
        assertEquals("3", leader.staticId);
        assertEquals(5.0, leader.commandSpeed, 0.01);
        assertEquals(5.0, leader.vehicleSpeed, 0.01);
        assertEquals(45.0, leader.vehiclePosition, 0.01);
    }
    
    @Test
    public void caseTwoInAPF() {
        // assign to the preceding vehicle when the gap in front is very small
        when(mockInputs.getDistanceFromRouteStart()).thenReturn(10.0);
        when(mockInputs.getDistanceToFrontVehicle()).thenReturn(1.0);
        when(mockRouteService.getCurrentDowntrackDistance()).thenReturn(10.0);
        mockPlugin.algorithmType = 1;
        manager.changeFromLeaderToFollower("A");
        manager.memberUpdates("3", "A", "00000000", "CMDSPEED:5.00,DTD:45.00,SPEED:5.00");
        manager.memberUpdates("2", "A", "00000000", "CMDSPEED:5.00,DTD:11.00,SPEED:5.00");
        manager.getLeader();
        PlatoonMember leader1 = manager.getLeader();
        assertEquals("2", leader1.staticId);
        assertEquals(5.0, leader1.commandSpeed, 0.01);
        assertEquals(5.0, leader1.vehicleSpeed, 0.01);
        assertEquals(11.0, leader1.vehiclePosition, 0.01);
        // assign to the preceding vehicle when the gap is still not large enough
        when(mockInputs.getDistanceToFrontVehicle()).thenReturn(1.99);
        manager.memberUpdates("2", "A", "00000000", "CMDSPEED:5.00,DTD:12.00,SPEED:5.00");
        //manager.platoon.get(1).vehiclePosition = 12.0;
        PlatoonMember leader2 = manager.getLeader();
        assertEquals("2", leader2.staticId);
        assertEquals(5.0, leader2.commandSpeed, 0.01);
        assertEquals(5.0, leader2.vehicleSpeed, 0.01);
        assertEquals(12.0, leader2.vehiclePosition, 0.01);
    }
    
    @Test
    public void caseThreeInAPF1() {
        // if there is no violation in time headway, we should follow the first vehicle
        when(mockInputs.getDistanceFromRouteStart()).thenReturn(10.0);
        when(mockInputs.getDistanceToFrontVehicle()).thenReturn(19.5);
        when(mockInputs.getCurrentSpeed()).thenReturn(10.0);
        mockPlugin.algorithmType = 1;
        manager.changeFromLeaderToFollower("A");
        when(mockRouteService.getCurrentDowntrackDistance()).thenReturn(10.0);
        manager.memberUpdates("3", "A", "00000000", "CMDSPEED:10.00,DTD:68.50,SPEED:10.00");
        manager.memberUpdates("2", "A", "00000000", "CMDSPEED:10.00,DTD:49.00,SPEED:10.00");
        manager.memberUpdates("1", "A", "00000000", "CMDSPEED:10.00,DTD:29.50,SPEED:10.00");
        manager.getLeader();
        PlatoonMember leader = manager.getLeader();
        assertEquals("3", leader.staticId);
        assertEquals(10.0, leader.commandSpeed, 0.01);
        assertEquals(10.0, leader.vehicleSpeed, 0.01);
        assertEquals(68.5, leader.vehiclePosition, 0.01);
    }
    
    @Test
    public void caseThreeInAPF2() {
        // if there is a violation in time headway, we should follow the vehicle caused that violation
        when(mockInputs.getDistanceFromRouteStart()).thenReturn(10.0);
        when(mockInputs.getDistanceToFrontVehicle()).thenReturn(19.5);
        when(mockRouteService.getCurrentDowntrackDistance()).thenReturn(10.0);
        when(mockInputs.getCurrentSpeed()).thenReturn(10.0);
        mockPlugin.algorithmType = 1;
        manager.changeFromLeaderToFollower("A");
        // there is a violation on max_spacing between the second vehicle and the third vehicle in this platoon
        manager.memberUpdates("3", "A", "00000000", "CMDSPEED:10.00,DTD:71.00,SPEED:10.00");
        manager.memberUpdates("2", "A", "00000000", "CMDSPEED:10.00,DTD:51.50,SPEED:10.00");
        manager.memberUpdates("1", "A", "00000000", "CMDSPEED:10.00,DTD:29.50,SPEED:10.00");
        manager.getLeader();
        PlatoonMember leader = manager.getLeader();
        assertEquals("1", leader.staticId);
        assertEquals(10.0, leader.commandSpeed, 0.01);
        assertEquals(10.0, leader.vehicleSpeed, 0.01);
        assertEquals(29.5, leader.vehiclePosition, 0.01);
    }
    
    @Test
    public void caseFourInAPF() {
        // not following the first vehicle in previous timesetp but change back to the first one when no more violation
        when(mockInputs.getDistanceFromRouteStart()).thenReturn(10.0);
        when(mockInputs.getDistanceToFrontVehicle()).thenReturn(18.0);
        when(mockInputs.getCurrentSpeed()).thenReturn(10.0);
        when(mockRouteService.getCurrentDowntrackDistance()).thenReturn(10.0);
        mockPlugin.algorithmType = 1;
        manager.changeFromLeaderToFollower("A");
        manager.memberUpdates("3", "A", "00000000", "CMDSPEED:5.00,DTD:45.00,SPEED:5.00");
        manager.memberUpdates("2", "A", "00000000", "CMDSPEED:5.00,DTD:11.00,SPEED:5.00");
        manager.getLeader();
        manager.getLeader();
        // there is a violation on max_spacing between the second vehicle and the third vehicle in this platoon
        manager.memberUpdates("3", "A", "00000000", "CMDSPEED:10.00,DTD:64.00,SPEED:10.00");
        manager.memberUpdates("2", "A", "00000000", "CMDSPEED:10.00,DTD:46.50,SPEED:10.00");
        manager.memberUpdates("1", "A", "00000000", "CMDSPEED:10.00,DTD:28.50,SPEED:10.00");
        PlatoonMember leader = manager.getLeader();
        assertEquals("3", leader.staticId);
        assertEquals(10.0, leader.commandSpeed, 0.01);
        assertEquals(10.0, leader.vehicleSpeed, 0.01);
        assertEquals(64.0, leader.vehiclePosition, 0.01);
    }
    
    @Test
    public void caseFiveInAPF() {
        // not following the first vehicle in previous timesetp and cannot change in this timestep
        when(mockInputs.getDistanceFromRouteStart()).thenReturn(10.0);
        when(mockInputs.getDistanceToFrontVehicle()).thenReturn(18.0);
        when(mockInputs.getCurrentSpeed()).thenReturn(10.0);
        when(mockRouteService.getCurrentDowntrackDistance()).thenReturn(10.0);
        mockPlugin.algorithmType = 1;
        manager.changeFromLeaderToFollower("A");
        manager.memberUpdates("3", "A", "00000000", "CMDSPEED:5.00,DTD:45.00,SPEED:5.00");
        manager.memberUpdates("2", "A", "00000000", "CMDSPEED:5.00,DTD:11.00,SPEED:5.00");
        manager.getLeader();
        manager.getLeader();
        // there is a violation on max_spacing between the second vehicle and the third vehicle in this platoon
        manager.memberUpdates("3", "A", "00000000", "CMDSPEED:10.00,DTD:63.00,SPEED:10.00");
        manager.memberUpdates("2", "A", "00000000", "CMDSPEED:10.00,DTD:45.00,SPEED:10.00");
        manager.memberUpdates("1", "A", "00000000", "CMDSPEED:10.00,DTD:28.00,SPEED:10.00");
        PlatoonMember leader = manager.getLeader();
        assertEquals("2", leader.staticId);
    }
    
}
