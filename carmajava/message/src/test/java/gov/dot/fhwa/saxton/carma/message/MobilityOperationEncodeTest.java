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

import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;
import org.ros.message.MessageFactory;

import cav_msgs.MobilityHeader;
import cav_msgs.MobilityOperation;
import gov.dot.fhwa.saxton.carma.message.factory.MobilityOperationMessage;
import gov.dot.fhwa.saxton.carma.rosutils.SaxtonLogger;

public class MobilityOperationEncodeTest {

    MobilityOperation         mockOperation;
    MobilityHeader            mockHeader;
    SaxtonLogger              mockLogger;
    MessageFactory            mockFactory;
    MobilityOperationMessage  message;
    
    @Before
    public void setup() {
        mockLogger = mock(SaxtonLogger.class);
        mockFactory = mock(MessageFactory.class);
        message = new MobilityOperationMessage(mockFactory, mockLogger);
        mockOperation = mock(MobilityOperation.class);
        mockHeader = mock(MobilityHeader.class);
        when(mockHeader.getSenderId()).thenReturn("USDOT-45100");
        when(mockHeader.getRecipientId()).thenReturn("USDOT-45095");
        when(mockHeader.getSenderBsmId()).thenReturn("10ABCDEF");
        when(mockHeader.getPlanId()).thenReturn("11111111-2222-3333-AAAA-111111111111");
        when(mockHeader.getTimestamp()).thenReturn(123456789L);
        when(mockOperation.getHeader()).thenReturn(mockHeader);
        when(mockOperation.getStrategy()).thenReturn("Carma/Platooning");
        when(mockOperation.getStrategyParams()).thenReturn("INFO|LEADER:USDOT-45100,REAR_DTD:34.22,SPEED:10.01");
    }
    
    @Test
    public void mobilityOperationEncodeTest() {
        long start = System.currentTimeMillis();
        byte[] res = message.callJniEncode(mockOperation);
        long end = System.currentTimeMillis();
        System.out.println("mobilityOperationEncodeTest takes " + (end - start) + "ms and the binary array length is " + res.length);
        byte[] expected = {0, -13, -128, -121, 77, 90, 113, 39, -44, 90, -47, -85, 22, 12, 38, -83, 56,
                           -109, -22, 45, 104, -43, -125, -106, -84, 88, 65, -123, 14, 36, 88, -52, 88,
                           -79, 98, -59, -117, 22, 43, 89, 50, 100, -55, 107, 54, 108, -39, -83, -125,
                           6, 12, 21, -84, 88, -79, 98, -59, -117, 22, 44, 88, -79, 98, -63, -125, 6,
                           12, 24, 48, 96, -63, -125, 22, 76, -38, 53, 108, -35, -61, -109, -95, -31,
                           -27, -73, 10, -6, 27, 48, -12, -33, -65, 118, -99, -39, -40, 73, -99, 26, 127,
                           -55, -111, 96, -60, -117, 73, -43, 90, 113, 39, -44, 90, -47, -85, 22, 12, 22,
                           82, -117, 6, -107, -8, -107, 34, 58, 102, -47, 115, 38, 75, 41, -48, -117, 22,
                           35, -90, 44, 23, 48, 98};
        Assert.assertArrayEquals(expected, res);
    }
}
