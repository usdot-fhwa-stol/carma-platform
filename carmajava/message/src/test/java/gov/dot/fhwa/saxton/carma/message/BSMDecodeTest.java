package gov.dot.fhwa.saxton.carma.message;

import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

import java.nio.ByteOrder;
import java.util.Arrays;

import javax.xml.bind.DatatypeConverter;

import org.jboss.netty.buffer.ChannelBuffer;
import org.jboss.netty.buffer.ChannelBuffers;
import org.junit.Before;
import org.junit.Test;
import org.ros.message.MessageFactory;
import org.ros.message.Time;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeConfiguration;

import gov.dot.fhwa.saxton.carma.message.factory.BSMMessage;
import gov.dot.fhwa.saxton.carma.rosutils.SaxtonLogger;

public class BSMDecodeTest {

    SaxtonLogger   mockLogger;
    BSMMessage    message;
    ConnectedNode mockNode;
    MessageFactory messageFactory = NodeConfiguration.newPrivate().getTopicMessageFactory();
    
    @Before
    public void setup() {
        mockLogger  = mock(SaxtonLogger.class);
        mockNode = mock(ConnectedNode.class);
        when(mockNode.getCurrentTime()).thenReturn(Time.fromMillis(0));
        message = new BSMMessage(mockNode, mockLogger, messageFactory);
    }
    
    /**
     * Test the decode bsm function
     * This test makes no assertions as to the results it only checks if the BSM could be decoded
     */
    @Test
    public void decodeBSM() {
        cav_msgs.ByteArray msg = messageFactory.newFromType(cav_msgs.ByteArray._TYPE);
        msg.setMessageType("BSM");
      
        String currentByteString = "00 14 25 03 97 0d 6b 3b 13 39 26 6e 92 6a 1e a6 c1 55 90 00 7f ff 8c cc af ff f0 80 7e fa 1f a1 00 7f ff 08 00 4b 09 b0";
        msg.setContent(getBufferFromHex(currentByteString));

        message.decode(msg);
    }

    /**
     * Helper function converts a Hex string to a ChannelBuffer for use in ROS messages
     * 
     * @param currentByteString The hex string to convert. Format "00 14 25"
     * 
     * @return A ChannelBuffer containing the matching binary data
     */
    ChannelBuffer getBufferFromHex(String currentByteString) {
        // All non hex characters are removed. This does not support use of x such as 0x00
        currentByteString = currentByteString.replaceAll("[^A-Fa-f0-9]", "");
    
        // An uneven number of characters will have a 0 appended to the end
        if (currentByteString.length() % 2 != 0) {
            currentByteString = currentByteString.concat("0");
        }
    
        // Convert the string to a byte array
        byte[] rawBytes = DatatypeConverter.parseHexBinary(currentByteString);
    
        return ChannelBuffers.copiedBuffer(ByteOrder.LITTLE_ENDIAN, rawBytes);
    }
}
