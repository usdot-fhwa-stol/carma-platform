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
        byte[] decodedMessage = {0, -16, -128, -122, 77, -72, -109, -22, 45, 104, -43, -117, 6, 23, 66,
                                 -35, -42, 44, 32, -62, -121, 18, 44, 102, 44, 88, -79, 98, -59, -117, 21,
                                 -84, -103, 50, 100, -75, -101, 54, 108, -42, -63, -125, 6, 10, -42, 44, 88,
                                 -79, 98, -59, -117, 22, 44, 88, -79, 96, -63, -125, 6, 12, 24, 48, 96, -63,
                                 -117, 38, 109, 26, -74, 110, -31, -56, 66, -36, 60, 60, -74, -31, 95, 67,
                                 102, 30, -101, -9, -18, -45, -69, 61, -48, -7, -45, 10, -111, 43, -90, 22,
                                 -44, 94, 76, 49, 12, -54, -63, -125, 6, 12, 24, 48, 96, -63, -125, 6, 12,
                                 24, 48, 96, -63, -125, 2, 118, -32, -46, -114, -59, -45, 85, -52, 22, 32,
                                 -125, 74, 59, 39, 76, 91, 46, 97, 116};
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
        byte[] decodedMessage = {0, -16, -127, -107, 77, -72, -109, -22, 45, 104, -43, -117, 6, 23, 66, -35, -42,
                                 44, 32, -62, -121, 18, 44, 102, 44, 88, -79, 98, -59, -117, 21, -84, -103, 50, 100,
                                 -75, -101, 54, 108, -42, -63, -125, 6, 10, -42, 44, 88, -79, 98, -59, -117, 22,
                                 44, 88, -79, 96, -63, -125, 6, 12, 24, 48, 96, -63, -117, 38, 109, 26, -74, 110,
                                 -31, -49, 66, -36, 60, 60, -74, -31, 95, 67, 102, 30, -101, -9, -18, -45, -69, 61,
                                 -48, -7, -45, 10, -111, 43, -90, 22, -44, 94, 76, 49, 12, -54, -63, -125, 6, 12,
                                 24, 48, 96, -63, -125, 6, 12, 24, 48, 96, -63, -125, 2, 118, -32, -46, -114, -59, -45,
                                 85, -52, 22, 32, -125, 74, 59, 39, 76, 91, 46, 97, 117, 48, 101, 68, -62, 96, -54,
                                 -119, -124, -63, -107, 19, 12, 24, 48, 96, -63, -125, 6, 12, 24, 48, 96, -63, -125,
                                 6, 12, 24, 53, -15, -12, 125, 31, 71, -43, -11, 125, 95, 103, -39, -10, 125, -33, 119,
                                 -35, -8, 126, 31, -121, -27, -7, 126, 95, -89, -23, -6, 126, -33, -73, -19, -4, 127, 31,
                                 -57, -11, -3, 127, 95, -25, -7, -2, 127, -33, -9, -2, 0, -128, 32, 8, 6, 1, -128, 96, 40,
                                 10, 2, -128, -32, 56, 14, 4, -127, 32, 72, 22, 5, -127, 96, 104, 26, 6, -127, -32, 120,
                                 30, 8, -126, 32, -120, 38, 9, -126, 96, -88, 42, 10, -126, -32, -72, 46, 12, -125, 32,
                                 -56, 54, 13, -125, 96, -24, 58, 14, -125, -32, -8, 62, 16, -124, 33, 8, 70, 17, -124, 97,
                                 40, 74, 18, -124, -31, 56, 78, 20, -123, 33, 72, 86, 21, -123, 97, 104, 90, 22, -123, -31,
                                 120, 94, 24, -122, 33, -120, 102, 25, -122, 97, -88, 106, 26, -122, -31, -72, 110, 28,
                                 -121, 33, -56, 118, 29, -121, 97, -24, 122, 30, -121, -31, -8, 126, 32, -120, 34, 8, -122,
                                 33, -120, 98, 40, -118, 34, -120, -30, 56, -114, 36, -119, 34, 72, -106, 37, -119, 98,
                                 104, -102, 38, -119, -30, 120, -98, 40, -118, 34, -120, -90, 41, -118, 98, -88, -86, 42,
                                 -118, -30, -72, -82, 44, -117, 34, -56, -74, 45, -117, 98, -24, -70, 46, -117, -30, -8,
                                 -67, -125, 6, 12, 24, 48, 96, -63, -125, 6, 12, 24, 48, 96, -63, -125, -122, 64};
        long start = System.currentTimeMillis();
        int res = message.callJniDecode(decodedMessage, request, senderId, targetId, bsmId, planId, timestamp, strategy,
                mock(PlanType.class), mock(LocationECEF.class), locationTime, strategyParams,
                mock(LocationECEF.class), trajectoryStartTime, offsets, expiration);
        long end = System.currentTimeMillis();
        System.out.println("decodeMobilityRequestWithOptionalField take " + (end - start) + "ms to finish.");
        assertEquals(0, res);
    }
}
