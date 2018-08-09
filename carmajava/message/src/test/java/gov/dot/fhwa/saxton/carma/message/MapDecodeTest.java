package gov.dot.fhwa.saxton.carma.message;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.mockito.Mockito.mock;

import java.util.Arrays;

import org.junit.Before;
import org.junit.Test;
import org.ros.message.MessageFactory;

import cav_msgs.MapData;
import gov.dot.fhwa.saxton.carma.message.factory.MapMessage;
import gov.dot.fhwa.saxton.carma.rosutils.SaxtonLogger;

public class MapDecodeTest {
    
    SaxtonLogger   mockLogger;
    MessageFactory mockFactory;
    MapMessage     message;
    MapData        mapData;
    
    @Before
    public void setup() {
        mockLogger  = mock(SaxtonLogger.class);
        mockFactory = mock(MessageFactory.class);
        message     = new MapMessage(mockFactory, mockLogger);
        mapData     = mock(MapData.class);
    }
    
    @Test
    public void decodeMap() {
        String hexMessage = "0012815338033020204bda0d4cdcf8143d4dc48811860224164802280008002297d4bc80a0a0a9825825923a90b2f2e418986f41b7006480602403812020084015480010004521d9f001414160c7c42a1879858619502a42a060e927100662000400105be6bf41c8aded5816ebc050507dcb860ec57aead5079e02828900890001000417223a50728b750f9c6ea9e8ae480a0a0f68746ad447c002828900a0880704404020803b9000200062b68d5305d1f9269a725027d8352f72867d6c82403340004000c53f5b761abbb7d35d3c0813ec1a3baac16bfc048050240301202008402208001000310fe55f849acd608d8ace136b440000dfe4808880008002086365c0017d1612eb34026067404895390907bd848050440302201c100024000200000090026180a0a0f2852600140001000000169fc1585bd1da000b00008000000a3bb2f439459a80060000400000046d55c416c67f40";
        byte[] byteMessage = new byte[hexMessage.length() / 2];
        for(int i = 0; i < byteMessage.length; i++) {
            byteMessage[i] = Integer.valueOf(hexMessage.substring(i * 2, i * 2 + 2), 16).byteValue();
        }
        System.out.println(Arrays.toString(byteMessage));
        int[] intersectionData = new int[9];
        int[] laneIDData = new int[255];
        int[] ingressApproachData = new int[255];
        int[] egressApproachData = new int[255];
        int[] laneDirectionData = new int[255];
        int[] laneTypeData = new int[255];
        int[][] nodeOffsetData = new int[255][189];
        int res = message.decodeMap(byteMessage, mapData, intersectionData, laneIDData, ingressApproachData,
                                    egressApproachData, laneDirectionData, laneTypeData, nodeOffsetData);
        // verify the decode is successful
        assertEquals(0, res);
        // intersectionId = 9709, revision = 3, lat = 389549844, long = -771493239
        // elevation = 390, elevationExist = 1, laneWidth = 274, laneWidthExist = 1, intersectionExist = 1 
        int[] intersection = {9709, 3, 389549844, -771493239, 390, 1, 274, 1, 1};
        assertArrayEquals(intersection, intersectionData);
        // In the test message, we have 12 lanes: 1, 5, 6, 2, 7, 3, 8, 4, 9, 10, 11, 12
        int[] laneId = new int[255];
        Arrays.fill(laneId, -1);
        int[] laneIdTemp = {1, 5, 6, 2, 7, 3, 8, 4, 9, 10, 11, 12};
        for(int i = 0; i < laneIdTemp.length; i++) {
            laneId[i] = laneIdTemp[i];
        }
        assertArrayEquals(laneId, laneIDData);
        // ingress approach lane for those lanes are: 1, not-exist, not-exist, 2, not-exist, 3, not-exist, 4, not-exist, not-exist, not-exist, not-exist
        int[] ingressApproach = new int[255];
        int[] ingressApproachTemp = {1, -1, -1, 2, -1, 3, -1, 4, -1, -1, -1, -1};
        for(int i = 0; i < ingressApproachTemp.length; i++) {
            ingressApproach[i] = ingressApproachTemp[i];
        }
        assertArrayEquals(ingressApproach, ingressApproachData);
        // egress approach lane for those lanes are: not-exist, 5, 6, not-exist, 7, not-exist, 8, not-exist, not-exist, not-exist, not-exist, not-exists
        int[] egressApproach = new int[255];
        int[] egressApproachTemp = {-1, 5, 6, -1, 7, -1, 8, -1, -1, -1, -1, -1};
        for(int i = 0; i < egressApproachTemp.length; i++) {
            egressApproach[i] = egressApproachTemp[i];
        }
        assertArrayEquals(egressApproach, egressApproachData);
        // directional use for those lanes are: ingressPath, egressPath, egressPath, ingressPath, egressPath,
        // ingressPath, egressPath, ingressPath, not-exists, not-exists, not-exists, not-exists
        int[] laneDirection = new int[255];
        int[] laneDirectionTemp = {128, 64, 64, 128, 64, 128, 64, 128};
        for(int i = 0; i < laneDirectionTemp.length; i++) {
            laneDirection[i] = laneDirectionTemp[i];
        }
        assertArrayEquals(laneDirection, laneDirectionData);
        // lane type: the first 8 lanes are for vehicle, the last 4 lanes are crosswalk
        int[] laneType = new int[255];
        int[] laneTypeTemp = {1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2};
        for(int i = 0; i < laneTypeTemp.length; i++) {
            laneType[i] = laneTypeTemp[i];
        }
        assertArrayEquals(laneType, laneTypeData);
        // TODO The nodeOffsetData array has too many data fields, I only verified the result manually
        System.out.println(Arrays.toString(nodeOffsetData[0]));
    }
    
}
