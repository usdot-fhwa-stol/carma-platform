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

package gov.dot.fhwa.saxton.carma.rsumetering;

import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;

import org.apache.commons.logging.Log;
import org.junit.Before;
import org.junit.Test;
import org.ros.message.MessageFactory;
import org.ros.node.NodeConfiguration;

import cav_msgs.MobilityOperation;
import gov.dot.fhwa.saxton.carma.rosutils.SaxtonLogger;

// This test only focus on the behavior of CommandingState API.
public class CommandingStateTest {
    protected SaxtonLogger                  mockLog;
    protected Log                           mockSimpleLog;
    protected RSUMeterWorker                mockRSUMeterWorker;

    NodeConfiguration nodeConfiguration = NodeConfiguration.newPrivate();
    MessageFactory messageFactory = nodeConfiguration.getTopicMessageFactory();

    final String VEHICLE_ID = "veh_id";
    final String BROADCAST_ID = "";
    final String RSU_ID = "rsu_id";
    final double vehMaxAccel = 0.0;
    final double distToMerge = 0.0;
    final double initialTargetSpeed = 0.0;

    @Before
    public void setup() {
        mockLog                 = mock(SaxtonLogger.class);
        mockSimpleLog           = mock(Log.class);
        mockRSUMeterWorker      = mock(RSUMeterWorker.class);
    }

    @Test
    public void testOnMobilityOperationMessage() {

        String planId = "AA-BB";
        final CommandingState commandingState = new CommandingState( mockRSUMeterWorker, mockLog, VEHICLE_ID, planId, vehMaxAccel, distToMerge, initialTargetSpeed);
        
        // Initialize message
        MobilityOperation msg = messageFactory.newFromType(MobilityOperation._TYPE);
        msg.getHeader().setRecipientId(VEHICLE_ID);
        msg.getHeader().setSenderId(VEHICLE_ID);
        msg.getHeader().setPlanId(planId);

        msg.setStrategyParams(String.format(""));
        // Execute function
        commandingState.onMobilityOperationMessage(msg);
        verify(mockLog , times(1)).warn("Received operation message with bad params. Exception: java.lang.IllegalArgumentException: Invalid type. Expected: STATUS String: ");

        msg.setStrategyParams(String.format("STATUS|METER_DIST:%.2f,MERGE_DIST:%.2f,SPEED:%.2f,LANE:%d", -11.00, -801.0, -0.6, 0));
        // Execute function
        commandingState.onMobilityOperationMessage(msg);
        verify(mockLog , times(1)).warn("Received operation message with suspect strategy variables. meterDist = -11.0, mergeDist = -801.0, speed = -0.6, lane = 0");

        msg.setStrategyParams(String.format("STATUS|METER_DIST:%.2f,MERGE_DIST:%.2f,SPEED:%.2f,LANE:%d", 1001.00, 801.0, 36.0, 0));
        // Execute function
        commandingState.onMobilityOperationMessage(msg);
        verify(mockLog , times(1)).warn("Received operation message with suspect strategy variables. meterDist = 1001.0, mergeDist = 801.0, speed = 36.0, lane = 0");

    }
}