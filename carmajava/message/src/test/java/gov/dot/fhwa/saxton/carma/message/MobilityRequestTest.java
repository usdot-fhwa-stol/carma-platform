package gov.dot.fhwa.saxton.carma.message;

import static org.junit.Assert.assertArrayEquals;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

import java.util.Arrays;
import java.util.LinkedList;

import org.junit.Before;
import org.junit.Test;
import org.ros.message.MessageFactory;

import cav_msgs.LocationECEF;
import cav_msgs.LocationOffsetECEF;
import cav_msgs.MobilityHeader;
import cav_msgs.MobilityRequest;
import cav_msgs.PlanType;
import cav_msgs.Trajectory;
import gov.dot.fhwa.saxton.carma.message.factory.MobilityRequestMessage;
import gov.dot.fhwa.saxton.carma.rosutils.SaxtonLogger;

public class MobilityRequestTest {
    
    MobilityRequest mockRequest;
    MobilityHeader mockHeader;
    PlanType mockType;
    LocationECEF currentLocation;
    Trajectory mockTrajectory;
    SaxtonLogger mockLogger;
    MessageFactory mockFactory;
    MobilityRequestMessage message;
    
    @Before
    public void setup() {
        mockLogger = mock(SaxtonLogger.class);
        mockFactory = mock(MessageFactory.class);
        message = new MobilityRequestMessage(mockLogger, mockFactory);
        mockRequest = mock(MobilityRequest.class);
        mockHeader = mock(MobilityHeader.class);
        when(mockHeader.getSenderId()).thenReturn("DOT-45100");
        when(mockHeader.getRecipientId()).thenReturn("");
        when(mockHeader.getSenderBsmId()).thenReturn("10ABCDEF");
        when(mockHeader.getPlanId()).thenReturn("11111111-2222-3333-AAAA-111111111111");
        when(mockHeader.getTimestamp()).thenReturn(123456789L);
        when(mockRequest.getHeader()).thenReturn(mockHeader);
        when(mockRequest.getStrategy()).thenReturn("Carma/Platooning");
        mockType = mock(PlanType.class);
        when(mockType.getType()).thenReturn((byte) 0);
        when(mockRequest.getPlanType()).thenReturn(mockType);
        when(mockRequest.getUrgency()).thenReturn((short) 999);
        currentLocation = mock(LocationECEF.class);
        when(currentLocation.getEcefX()).thenReturn(555555);
        when(currentLocation.getEcefY()).thenReturn(666666);
        when(currentLocation.getEcefZ()).thenReturn(777777);
        when(currentLocation.getTimestamp()).thenReturn(0L);
        when(mockRequest.getLocation()).thenReturn(currentLocation);
        when(mockRequest.getStrategyParams()).thenReturn("ARG1:5.0, ARG2:16.0");
        mockTrajectory = mock(Trajectory.class);
        when(mockTrajectory.getLocation()).thenReturn(currentLocation);
        when(mockTrajectory.getOffsets()).thenReturn(new LinkedList<LocationOffsetECEF>());
        when(mockRequest.getTrajectory()).thenReturn(mockTrajectory);
        when(mockRequest.getExpiration()).thenReturn(0L);
    }
    
    @Test
    public void mobilityRequestEncodeTest1() {
        byte[] res = message.callJniEncode(mockRequest);
        byte[] expected = {0, -16, -128, -122, 77, -72, -109, -22, 45, 104, -43, -117, 6, 23, 66,
                           -35, -42, 44, 32, -62, -121, 18, 44, 102, 44, 88, -79, 98, -59, -117, 21,
                           -84, -103, 50, 100, -75, -101, 54, 108, -42, -63, -125, 6, 10, -42, 44, 88,
                           -79, 98, -59, -117, 22, 44, 88, -79, 96, -63, -125, 6, 12, 24, 48, 96, -63,
                           -117, 38, 109, 26, -74, 110, -31, -56, 66, -36, 60, 60, -74, -31, 95, 67,
                           102, 30, -101, -9, -18, -45, -69, 61, -48, -7, -45, 12, 67, 50, -90, 22, -44,
                           94, 76, 25, 80, 104, -63, -125, 6, 12, 24, 48, 96, -63, -125, 6, 12, 24, 48, 96,
                           -63, -125, 2, 118, -32, -46, -114, -59, -45, 85, -52, 22, 32, -125, 74, 59,
                           39, 76, 91, 46, 97, 116};
        assertArrayEquals(expected, res);
    }
}
