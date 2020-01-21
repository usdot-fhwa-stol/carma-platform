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

import cav_msgs.MobilityResponse;
import gov.dot.fhwa.saxton.carma.message.factory.MobilityResponseMessage;
import gov.dot.fhwa.saxton.carma.rosutils.SaxtonLogger;

public class MobilityResponseDecodeTest {
    
    SaxtonLogger            mockLogger;
    MessageFactory          mockFactory;
    MobilityResponseMessage message;
    MobilityResponse        request;
    
    @Before
    public void setup() {
        mockLogger = mock(SaxtonLogger.class);
        mockFactory = mock(MessageFactory.class);
        message = new MobilityResponseMessage(mockLogger, mockFactory);
        request = mock(MobilityResponse.class);
    }
    
    @Test
    public void decodeMobilityResponse() {
        byte[] senderId = new byte[16];
        byte[] targetId = new byte[16];
        byte[] bsmId = new byte[8];
        byte[] planId = new byte[36];
        byte[] timestamp = new byte[19];
        byte[] encodedMessage = {0, -15, 77, 77, 90, 113, 39, -44, 90, -47, -85, 22, 12, 38, -83, 56, -109,
                -22, 45, 104, -43, -125, -106, -84, 88, 65, -123, 14, 36, 88, -52, 88, -79,
                98, -59, -117, 22, 43, 89, 50, 100, -55, 107, 54, 108, -39, -83, -125, 6,
                12, 21, -84, 88, -79, 98, -59, -117, 22, 44, 88, -79, 98, -63, -125, 6, 12,
                24, 48, 96, -63, -125, 22, 76, -38, 53, 108, -35, -61, -112, -54};
        long start = System.currentTimeMillis();
        int res = message.decodeMobilityResponse(encodedMessage, request, senderId, targetId, bsmId, planId, timestamp);
        long end = System.currentTimeMillis();
        System.out.println("decodeMobilityResponse take " + (end - start) + "ms to finish.");
        assertEquals(0, res);
    }
}
