package gov.dot.fhwa.saxton.carma.message;

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
        int[] intersectionData = new int[6];
        int res = message.decodeMap(byteMessage, mapData, intersectionData);
        assertEquals(0, res);
        System.out.println(Arrays.toString(intersectionData));
        // verify some fields
    }

}
