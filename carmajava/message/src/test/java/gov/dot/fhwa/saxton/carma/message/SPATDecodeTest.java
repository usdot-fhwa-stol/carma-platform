package gov.dot.fhwa.saxton.carma.message;

import static org.junit.Assert.assertEquals;
import static org.mockito.Mockito.mock;

import java.util.Arrays;

import org.junit.Before;
import org.junit.Test;
import org.ros.message.MessageFactory;

import gov.dot.fhwa.saxton.carma.message.factory.SPATMessage;
import gov.dot.fhwa.saxton.carma.rosutils.SaxtonLogger;

public class SPATDecodeTest {

    SaxtonLogger   mockLogger;
    MessageFactory mockFactory;
    SPATMessage    message;
    
    @Before
    public void setup() {
        mockLogger  = mock(SaxtonLogger.class);
        mockFactory = mock(MessageFactory.class);
        message = new SPATMessage(mockFactory, mockLogger);
    }
    
    @Test
    public void decodeMap() {
        String hexMessage = "00131c44630800812f6800000c2d40100204342b3fac0a0020232159495f9c";
        byte[] byteMessage = new byte[hexMessage.length() / 2];
        for(int i = 0; i < byteMessage.length; i++) {
            byteMessage[i] = Integer.valueOf(hexMessage.substring(i * 2, i * 2 + 2), 16).byteValue();
        }
        System.out.println(Arrays.toString(byteMessage));
        int[] intersectionData = new int[8];
        int[][] movementStatesData = new int[255][145];
        int res = message.decodeSPAT(byteMessage, intersectionData, movementStatesData);
        assertEquals(0, res);
        // TODO Because we do not know the expect values, we can only evaluate data roughly
        System.out.println(Arrays.toString(intersectionData));
        System.out.println(Arrays.toString(movementStatesData[0]));
        System.out.println(Arrays.toString(movementStatesData[1]));
        System.out.println(Arrays.toString(movementStatesData[2]));
    }
}
