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

import java.util.List;

import static org.junit.Assert.assertEquals;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;
import static org.mockito.Matchers.any;
import static org.mockito.Matchers.anyString;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.apache.commons.logging.Log;
import org.junit.Before;
import org.junit.Test;
import org.mockito.ArgumentCaptor;
import org.ros.message.MessageFactory;
import org.ros.node.NodeConfiguration;

import cav_msgs.BSM;
import cav_msgs.MobilityOperation;
import cav_msgs.MobilityRequest;
import cav_msgs.MobilityResponse;
import gov.dot.fhwa.saxton.carma.geometry.geodesic.Location;
import gov.dot.fhwa.saxton.carma.rosutils.MobilityHelper;
import gov.dot.fhwa.saxton.carma.rosutils.SaxtonLogger;

// This test only focus on the behavior of CommandingState API.
public class RSUMeterWorkerTest {
    protected SaxtonLogger                  mockLog;
    protected Log                           mockSimpleLog;
    protected RSUMeterWorker                mockRSUMeterWorker;
    protected IRSUMeterManager              mockIRSUMeterManager;

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
        mockIRSUMeterManager      = mock(IRSUMeterManager.class);
    }

    @Test
    public void testonUpdatePlatoonWithOperationMsg() {

        String planId = "AA-BB";
        double vehLagTime = 1.0;

        String routeFilePath = "/home/carma/src/CARMAPlatform/carmajava/route/src/test/resources/routes/25_glidepath_demo_east_bound.yaml";
        String rsuId = "veh_id";
        double distToMerge = 0; 
        double mainRouteMergeDTD = 0; 
        double meterRadius = 0;
        int targetLane = 0;
        double mergeLength = 0; 
        long timeMargin = 0;
        long requestPeriod = 0; 
        long commandPeriod = 0; 
        long commsTimeout = 0;
        Location meterLoc = new Location(0, 0, 0);
        double minApproachAccel = 0; 
        double targetApproachSpeed = 0; 
        double driverLagTime = 0;
        double commsLagTime = 0;

        when(mockLog.getBaseLoggerObject()).thenReturn(mock(Log.class));

        final RSUMeterWorker rSUMeterWorker = new RSUMeterWorker( mockIRSUMeterManager, mockLog, routeFilePath, rsuId,  distToMerge,  mainRouteMergeDTD,  meterRadius, targetLane,  mergeLength,  timeMargin, requestPeriod,  commandPeriod,  commsTimeout,  meterLoc, minApproachAccel,  targetApproachSpeed,  driverLagTime,  commsLagTime);

        // Initialize message
        MobilityOperation  msg = messageFactory.newFromType(MobilityOperation._TYPE);
        msg.getHeader().setRecipientId(VEHICLE_ID);
        msg.getHeader().setSenderId(VEHICLE_ID);
        msg.getHeader().setPlanId(planId);
        msg.setStrategy("Carma/Platooning");
        msg.setStrategyParams(String.format(""));
        // Execute function
        rSUMeterWorker.handleMobilityOperationMsg(msg);
        verify(mockLog, times(1)).warn("Received operation message with bad params. Exception: java.lang.IllegalArgumentException: Invalid type. Expected: STATUS String: ");

        msg.setStrategyParams(String.format("INFO|METER_DIST:%.2f,MERGE_DIST:%.2f,SPEED:%.2f,LANE:%d", -11.00, -801.0, -0.6, 0));
        // Execute function
        rSUMeterWorker.handleMobilityOperationMsg(msg);
        verify(mockLog, times(1)).warn("Received operation message with suspect strategy values. meterDist = -11.0, mergeDist = -801.0, speed = -0.6");

        msg.setStrategyParams(String.format("INFO|METER_DIST:%.2f,MERGE_DIST:%.2f,SPEED:%.2f,LANE:%d", 1001.0, 801.0, 36.0, 0));
        // Execute function
        rSUMeterWorker.handleMobilityOperationMsg(msg);
        verify(mockLog , times(1)).warn("Received operation message with suspect strategy values. meterDist = 1001.0, mergeDist = 801.0, speed = 36.0");

    }
}