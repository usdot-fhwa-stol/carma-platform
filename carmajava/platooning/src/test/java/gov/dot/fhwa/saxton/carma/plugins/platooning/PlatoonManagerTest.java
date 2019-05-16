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

import gov.dot.fhwa.saxton.carma.guidance.mobilityrouter.IMobilityRouter;
import gov.dot.fhwa.saxton.carma.guidance.plugins.PluginServiceLocator;
import gov.dot.fhwa.saxton.carma.guidance.util.ILogger;

public class PlatoonManagerTest {

    private PlatoonManager       manager;
    private PlatooningPlugin     mockPlugin;
    private ILogger              mockLogger;
    private PluginServiceLocator mockPsl;
    private IMobilityRouter      mockRouter;
    private Clock                mockClock;
    
    @Before
    public void setup() {
        mockLogger = mock(ILogger.class);
        mockPlugin = mock(PlatooningPlugin.class);
        mockPsl    = mock(PluginServiceLocator.class);
        mockClock  = mock(Clock.class);
        mockRouter = mock(IMobilityRouter.class);
        when(mockPsl.getMobilityRouter()).thenReturn(mockRouter);
        mockPlugin.statusTimeoutFactor = 2.5;
        manager    = new PlatoonManager(mockPlugin, mockLogger, mockPsl, mockClock);
    }
    
    @Test
    public void updateMemberInfoInLeaderState() {
        assertNull(manager.getLeader());
        manager.memberUpdates("A", manager.currentPlatoonID, "00000000", "CMDSPEED:1.00,DTD:50.00,SPEED:1.00");
        assertNull(manager.getLeader());
        assertEquals(2, manager.getTotalPlatooningSize());
        assertEquals(50.0, manager.getPlatoonRearDowntrackDistance(), 0.01);
        manager.memberUpdates("A", manager.currentPlatoonID, "00000000", "CMDSPEED:1.00,DTD:60.00,SPEED:1.00");
        assertEquals(2, manager.getTotalPlatooningSize());
        assertEquals(60.0, manager.getPlatoonRearDowntrackDistance(), 0.01);
        manager.memberUpdates("B", manager.currentPlatoonID, "00000001", "CMDSPEED:1.00,DTD:70.00,SPEED:1.00");
        assertEquals(3, manager.getTotalPlatooningSize());
        assertEquals(60.0, manager.getPlatoonRearDowntrackDistance(), 0.01);
        manager.memberUpdates("C", manager.currentPlatoonID, "00000002", "CMDSPEED:1.00,DTD:10.00,SPEED:1.00");
        assertEquals(4, manager.getTotalPlatooningSize());
        assertEquals(10.0, manager.getPlatoonRearDowntrackDistance(), 0.01);
    }
    
    @Test
    public void removeExpiredMember() {
    	when(mockClock.millis()).thenReturn(1549901201448L, 1549901201448L, 1549901201699L);
        manager.memberUpdates("A", manager.currentPlatoonID, "00000000", "CMDSPEED:1.00,DTD:50.00,SPEED:1.00");
        assertEquals(2, manager.getTotalPlatooningSize());
        manager.removeExpiredMember();
        assertEquals(2, manager.getTotalPlatooningSize());
        manager.removeExpiredMember();
        assertEquals(1, manager.getTotalPlatooningSize());
    }
    
}
