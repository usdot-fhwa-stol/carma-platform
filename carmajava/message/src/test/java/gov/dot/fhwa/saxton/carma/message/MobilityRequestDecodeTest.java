/*
 * Copyright (C) 2018-2020 LEIDOS.
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

package gov.dot.fhwa.saxton.carma.message;

import static org.junit.Assert.assertEquals;
import static org.mockito.Mockito.mock;

import org.junit.Before;
import org.junit.Test;
import org.ros.message.MessageFactory;

import cav_msgs.LocationECEF;
import cav_msgs.MobilityRequest;
import cav_msgs.PlanType;
import gov.dot.fhwa.saxton.carma.message.factory.MobilityRequestMessage;
import gov.dot.fhwa.saxton.carma.rosutils.SaxtonLogger;

public class MobilityRequestDecodeTest {

    SaxtonLogger mockLogger;
    MessageFactory mockFactory;
    MobilityRequestMessage message;
    MobilityRequest request;
    
    @Before
    public void setup() {
        mockLogger = mock(SaxtonLogger.class);
        mockFactory = mock(MessageFactory.class);
        message = new MobilityRequestMessage(mockLogger, mockFactory);
        request = mock(MobilityRequest.class);
    }
    
    @Test
    public void decodeMobilityRequestWithoutOptionalField() {
        byte[] senderId = new byte[16];
        byte[] targetId = new byte[16];
        byte[] bsmId = new byte[8];
        byte[] planId = new byte[36];
        byte[] timestamp = new byte[19];
        byte[] strategy = new byte[50];
        byte[] locationTime = new byte[19];
        byte[] strategyParams = new byte[100];
        byte[] trajectoryStartTime = new byte[19];
        int[][] offsets = new int[3][60];
        byte[] expiration = new byte[19];
        byte[] decodedMessage = {0, -16, -128, -125, 77, 90, 113, 39, -44, 90, -47, -85, 22, 12, 2, -35,
                                 -42, 44, 32, -62, -121, 18, 44, 102, 44, 88, -79, 98, -59, -117, 21, -84,
                                 -103, 50, 100, -75, -101, 54, 108, -42, -63, -125, 6, 10, -42, 44, 88, -79,
                                 98, -59, -117, 22, 44, 88, -79, 96, -63, -125, 6, 12, 24, 48, 96, -63, -117,
                                 38, 109, 26, -74, 110, -31, -56, 58, 30, 30, 91, 112, -81, -95, -77, 15, 77,
                                 -5, -9, 105, -35, -100, 62, 116, -62, -92, 74, -23, -123, -75, 23, -109, 12,
                                 67, 50, -80, 96, -63, -125, 6, 12, 24, 48, 96, -63, -125, 6, 12, 24, 48, 96,
                                 -64, -116, 26, 81, -40, -70, 106, -71, -126, -60, 16, 105, 71, 100, -23, -117, 101, -52, 0};
        long start = System.currentTimeMillis();
        int res = message.callJniDecode(decodedMessage, request, senderId, targetId, bsmId, planId, timestamp, strategy,
                                        mock(PlanType.class), mock(LocationECEF.class), locationTime, strategyParams,
                                        mock(LocationECEF.class), trajectoryStartTime, offsets, expiration);
        long end = System.currentTimeMillis();
        System.out.println("decodeMobilityRequestWithoutOptionalField take " + (end - start) + "ms to finish.");
        assertEquals(0, res);
    }
    
    @Test
    public void decodeMobilityRequestWithOptionalField() {
        byte[] senderId = new byte[16];
        byte[] targetId = new byte[16];
        byte[] bsmId = new byte[8];
        byte[] planId = new byte[36];
        byte[] timestamp = new byte[19];
        byte[] strategy = new byte[50];
        byte[] locationTime = new byte[19];
        byte[] strategyParams = new byte[100];
        byte[] trajectoryStartTime = new byte[19];
        int[][] offsets = new int[3][60];
        byte[] expiration = new byte[19];
        byte[] decodedMessage = {0, -16, -127, -111, 77, 90, 113, 39, -44, 90, -47, -85, 22, 12, 2, -35, -42, 44,
                                 32, -62, -121, 18, 44, 102, 44, 88, -79, 98, -59, -117, 21, -84, -103, 50, 100,
                                 -75, -101, 54, 108, -42, -63, -125, 6, 10, -42, 44, 88, -79, 98, -59, -117, 22,
                                 44, 88, -79, 96, -63, -125, 6, 12, 24, 48, 96, -63, -117, 38, 109, 26, -74, 110,
                                 -31, -49, 58, 30, 30, 91, 112, -81, -95, -77, 15, 77, -5, -9, 105, -35, -100, 62,
                                 116, -62, -92, 74, -23, -123, -75, 23, -109, 12, 67, 50, -80, 96, -63, -125, 6, 12,
                                 24, 48, 96, -63, -125, 6, 12, 24, 48, 96, -64, -116, 26, 81, -40, -70, 106, -71, -126,
                                 -60, 16, 105, 71, 100, -23, -117, 101, -52, 19, 6, 84, 76, 38, 12, -88, -104, 76, 25,
                                 81, 48, -63, -125, 6, 12, 24, 48, 96, -63, -125, 6, 12, 24, 48, 96, -63, -125, 95, 31,
                                 71, -47, -12, 125, 95, 87, -43, -10, 125, -97, 103, -35, -9, 125, -33, -121, -31, -8,
                                 126, 95, -105, -27, -6, 126, -97, -89, -19, -5, 126, -33, -57, -15, -4, 127, 95, -41,
                                 -11, -2, 127, -97, -25, -3, -1, 127, -32, 8, 2, 0, -128, 96, 24, 6, 2, -128, -96, 40,
                                 14, 3, -128, -32, 72, 18, 4, -127, 96, 88, 22, 6, -127, -96, 104, 30, 7, -127, -32, -120,
                                 34, 8, -126, 96, -104, 38, 10, -126, -96, -88, 46, 11, -126, -32, -56, 50, 12, -125, 96,
                                 -40, 54, 14, -125, -96, -24, 62, 15, -125, -31, 8, 66, 16, -124, 97, 24, 70, 18, -124,
                                 -95, 40, 78, 19, -124, -31, 72, 82, 20, -123, 97, 88, 86, 22, -123, -95, 104, 94, 23,
                                 -123, -31, -120, 98, 24, -122, 97, -104, 102, 26, -122, -95, -88, 110, 27, -122, -31,
                                 -56, 114, 28, -121, 97, -40, 118, 30, -121, -95, -24, 126, 31, -121, -30, 8, -126, 32,
                                 -120, 98, 24, -122, 34, -120, -94, 40, -114, 35, -120, -30, 72, -110, 36, -119, 98, 88,
                                 -106, 38, -119, -94, 104, -98, 39, -119, -30, -120, -94, 40, -118, 98, -104, -90, 42,
                                 -118, -94, -88, -82, 43, -118, -30, -56, -78, 44, -117, 98, -40, -74, 46, -117, -94, -24,
                                 -66, 47, -117, -40, 48, 96, -63, -125, 6, 12, 24, 48, 96, -63, -125, 6, 12, 24, 56, 100};
        long start = System.currentTimeMillis();
        int res = message.callJniDecode(decodedMessage, request, senderId, targetId, bsmId, planId, timestamp, strategy,
                mock(PlanType.class), mock(LocationECEF.class), locationTime, strategyParams,
                mock(LocationECEF.class), trajectoryStartTime, offsets, expiration);
        long end = System.currentTimeMillis();
        System.out.println("decodeMobilityRequestWithOptionalField take " + (end - start) + "ms to finish.");
        assertEquals(0, res);
    }
}
