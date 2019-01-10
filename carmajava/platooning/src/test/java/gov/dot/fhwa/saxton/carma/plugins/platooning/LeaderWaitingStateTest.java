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
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

import java.util.concurrent.atomic.AtomicBoolean;

import org.junit.Before;
import org.junit.Test;

import cav_msgs.MobilityHeader;
import cav_msgs.MobilityRequest;
import cav_msgs.PlanType;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.IManeuverInputs;
import gov.dot.fhwa.saxton.carma.guidance.mobilityrouter.MobilityRequestResponse;
import gov.dot.fhwa.saxton.carma.guidance.plugins.PluginServiceLocator;
import gov.dot.fhwa.saxton.carma.guidance.util.ILogger;

public class LeaderWaitingStateTest {

    protected PlatooningPlugin     plugin;
    protected ILogger              log;
    protected PluginServiceLocator pluginServiceLocator;
    protected PlatoonManager       mockManager;
    protected IManeuverInputs      mockInputs;
    protected IPlatooningState     leaderWaitingState;
    
    @Before
    public void setup() {
        plugin = mock(PlatooningPlugin.class);
        log = mock(ILogger.class);
        pluginServiceLocator = mock(PluginServiceLocator.class);
        mockManager = mock(PlatoonManager.class);
        mockInputs = mock(IManeuverInputs.class);
        plugin.platoonManager = mockManager;
        when(plugin.getManeuverInputs()).thenReturn(mockInputs);
        plugin.handleMobilityPath = new AtomicBoolean(true);
        leaderWaitingState = new LeaderWaitingState(plugin, log, pluginServiceLocator, "C");
    }
    
    @Test
    public void onMobilityFollowerJoinRequestWithRightDistance() {
        MobilityRequest request = mock(MobilityRequest.class);
        MobilityHeader header = mock(MobilityHeader.class);
        PlanType type = mock(PlanType.class);
        when(header.getSenderId()).thenReturn("C");
        when(type.getType()).thenReturn(PlanType.PLATOON_FOLLOWER_JOIN);
        when(request.getHeader()).thenReturn(header);
        when(request.getPlanType()).thenReturn(type);
        when(request.getStrategyParams()).thenReturn("");
        when(mockManager.getPlatoonRearDowntrackDistance()).thenReturn(60.0);
        when(plugin.getDesiredJoinTimeGap()).thenReturn(4.0);
        when(mockInputs.getCurrentSpeed()).thenReturn(3.25);
        assertEquals(MobilityRequestResponse.ACK, leaderWaitingState.onMobilityRequestMessgae(request));
    }
    
}
