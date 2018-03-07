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

import cav_msgs.NewPlan;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.IManeuverInputs;
import gov.dot.fhwa.saxton.carma.guidance.util.ILogger;

public class PlatoonManagerTest {

    private PlatoonManager manager;
    private PlatooningPlugin mockPlugin;
    private ILogger mockLogger;
    
    @Before
    public void setup() {
        mockLogger = mock(ILogger.class);
        mockPlugin = mock(PlatooningPlugin.class);
        manager = new PlatoonManager(mockPlugin, mockLogger);
    }
    
    @Test
    public void findLeader() {
        // Return the leader with largest downtrack distance
        // TODO Once we integrate the leader selection algorithm, we need change this test case
        manager.platoon.add(new PlatoonMember("", 0, 0, 50, Long.MAX_VALUE));
        manager.platoon.add(new PlatoonMember("", 0, 0, 60, Long.MAX_VALUE));
        PlatoonMember leader = manager.getLeader();
        assertEquals(60.0, leader.getVehiclePosition(), 0.1);
    }
    
    @Test
    public void addNewMember() {
        // Add new member from new plan message of vehicles in front of us
        // TODO It might change to other message type
        NewPlan newPlan = mock(NewPlan.class);
        when(newPlan.getSenderId()).thenReturn("1");
        when(newPlan.getInputs()).thenReturn("CMDSPEED:5.0, DOWNTRACK:100.0, SPEED:4.9");
        NewPlan newPlan2 = mock(NewPlan.class);
        when(newPlan2.getSenderId()).thenReturn("2");
        when(newPlan2.getInputs()).thenReturn("CMDSPEED:6.0, DOWNTRACK:5.0, SPEED:5.9");
        IManeuverInputs maneuverInputs = mock(IManeuverInputs.class);
        when(maneuverInputs.getDistanceFromRouteStart()).thenReturn(50.0);
        when(mockPlugin.getManeuverInputs()).thenReturn(maneuverInputs);
        manager.memberUpdates(newPlan);
        manager.memberUpdates(newPlan2);
        assertEquals(1, manager.platoon.size());
        PlatoonMember desiredMember = new PlatoonMember("1", 5.0, 4.9, 100, 0);
        PlatoonMember actualMember = manager.platoon.first();
        assertEquals(desiredMember.getStaticId(), actualMember.getStaticId());
        assertEquals(desiredMember.getCommandSpeed(), actualMember.getCommandSpeed(), 0.01);
        assertEquals(desiredMember.getVehicleSpeed(), actualMember.getVehicleSpeed(), 0.01);
        assertEquals(desiredMember.getVehiclePosition(), actualMember.getVehiclePosition(), 0.01);
        assertEquals(System.currentTimeMillis(), actualMember.getTimestamp(), 1000);
    }
    
    @Test
    public void updateExistedMember() {
        // Update an existed member
        // TODO It might change to other message type
        PlatoonMember existedMember = new PlatoonMember("1", 0, 0, 5, 0);
        manager.platoon.add(existedMember);
        NewPlan newPlan = mock(NewPlan.class);
        when(newPlan.getSenderId()).thenReturn("1");
        when(newPlan.getInputs()).thenReturn("CMDSPEED:5.0, DOWNTRACK:100.0, SPEED:4.9");
        IManeuverInputs maneuverInputs = mock(IManeuverInputs.class);
        when(maneuverInputs.getDistanceFromRouteStart()).thenReturn(50.0);
        when(mockPlugin.getManeuverInputs()).thenReturn(maneuverInputs);
        manager.memberUpdates(newPlan);
        assertEquals(1, manager.platoon.size());
        PlatoonMember desiredMember = new PlatoonMember("1", 5.0, 4.9, 100, 0);
        PlatoonMember actualMember = manager.platoon.first();
        assertEquals(desiredMember.getStaticId(), actualMember.getStaticId());
        assertEquals(desiredMember.getCommandSpeed(), actualMember.getCommandSpeed(), 0.01);
        assertEquals(desiredMember.getVehicleSpeed(), actualMember.getVehicleSpeed(), 0.01);
        assertEquals(desiredMember.getVehiclePosition(), actualMember.getVehiclePosition(), 0.01);
        assertEquals(System.currentTimeMillis(), actualMember.getTimestamp(), 1000);
    }
}
