
package gov.dot.fhwa.saxton.carma.message;

import static org.junit.Assert.*;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

import org.junit.Before;
import org.junit.Ignore;
import org.junit.Test;
import org.ros.message.MessageFactory;

import cav_msgs.LocationECEF;
import cav_msgs.LocationOffsetECEF;
import cav_msgs.MobilityHeader;
import cav_msgs.MobilityPath;
import cav_msgs.MobilityRequest;
import cav_msgs.PlanType;
import cav_msgs.Trajectory;
import gov.dot.fhwa.saxton.carma.message.factory.MessageContainer;
import gov.dot.fhwa.saxton.carma.message.factory.MobilityPathMessage;
import gov.dot.fhwa.saxton.carma.message.factory.MobilityRequestMessage;
import gov.dot.fhwa.saxton.carma.rosutils.SaxtonLogger;

public class MobilityPathTest {

    MobilityPath mockPath;
    MobilityHeader mockHeader;
    LocationECEF mockStartLocation;
    Trajectory mockTrajectory;
    SaxtonLogger mockLogger;
    MessageFactory mockFactory;
    MobilityPathMessage message;

    @Before
    public void setup() {
        mockLogger = mock(SaxtonLogger.class);
        mockFactory = mock(MessageFactory.class);
        message = new MobilityPathMessage(mockFactory, mockLogger);
        mockPath = mock(MobilityPath.class);
        mockHeader = mock(MobilityHeader.class);
        when(mockHeader.getSenderId()).thenReturn("USDOT-45100");
        when(mockHeader.getRecipientId()).thenReturn("");
        when(mockHeader.getSenderBsmId()).thenReturn("10ABCDEF");
        when(mockHeader.getPlanId()).thenReturn("11111111-2222-3333-AAAA-111111111111");
        when(mockHeader.getTimestamp()).thenReturn(123456789L);
        when(mockPath.getHeader()).thenReturn(mockHeader);

        mockTrajectory = mock(Trajectory.class);
        mockStartLocation = mock(LocationECEF.class);
        when(mockStartLocation.getEcefX()).thenReturn(0);
        when(mockStartLocation.getEcefY()).thenReturn(0);
        when(mockStartLocation.getEcefZ()).thenReturn(0);
        when(mockTrajectory.getOffsets()).thenReturn(new LinkedList<LocationOffsetECEF>());
        when(mockTrajectory.getLocation()).thenReturn(mockStartLocation);
        when(mockPath.getTrajectory()).thenReturn(mockTrajectory);
    }

    @Test
    public void mobilityPathEncodeWithNoOffsets() {
        byte[] data = message.callJniEncode(mockPath);
        System.out.println(Arrays.toString(data));
        byte[] expected = { 0, -14, 97, 77, 90, 113, 39, -44, 90, -47, -85, 22, 12, 2, -35, -42, 44, 32, -62, -121, 18,
                44, 102, 44, 88, -79, 98, -59, -117, 21, -84, -103, 50, 100, -75, -101, 54, 108, -42, -63, -125, 6, 10,
                -42, 44, 88, -79, 98, -59, -117, 22, 44, 88, -79, 96, -63, -125, 6, 12, 24, 48, 96, -63, -117, 38, 109,
                26, -74, 110, -31, -54, 96, -54, -125, 68, -63, -107, 6, -119, -125, 42, 13, 24, 48, 96, -63, -125, 6,
                12, 24, 48, 96, -63, -125, 6, 12, 24, 48, 96, 0 };
        assertArrayEquals(expected, data);
    }

    @Test
    public void decodeMobilityRequestWithNoOffsets() {
        byte[] senderId = new byte[16];
        byte[] targetId = new byte[16];
        byte[] bsmId = new byte[8];
        byte[] planId = new byte[36];
        byte[] timestamp = new byte[19];
        byte[] trajectoryStartTime = new byte[19];
        int[][] offsets = new int[3][60];
        byte[] decodedMessage = { 0, -14, 97, 77, 90, 113, 39, -44, 90, -47, -85, 22, 12, 2, -35, -42, 44, 32, -62, -121, 18,
                44, 102, 44, 88, -79, 98, -59, -117, 21, -84, -103, 50, 100, -75, -101, 54, 108, -42, -63, -125, 6, 10,
                -42, 44, 88, -79, 98, -59, -117, 22, 44, 88, -79, 96, -63, -125, 6, 12, 24, 48, 96, -63, -117, 38, 109,
                26, -74, 110, -31, -54, 96, -54, -125, 68, -63, -107, 6, -119, -125, 42, 13, 24, 48, 96, -63, -125, 6,
                12, 24, 48, 96, -63, -125, 6, 12, 24, 48, 96, 0 };
        int res = message.decodeMobilityPath(decodedMessage, mockPath, senderId, targetId, bsmId, planId, timestamp, 
                                        mock(LocationECEF.class), trajectoryStartTime, offsets);
        assertEquals(0, res);
    }
}
