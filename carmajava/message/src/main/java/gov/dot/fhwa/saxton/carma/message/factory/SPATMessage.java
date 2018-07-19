package gov.dot.fhwa.saxton.carma.message.factory;

import org.ros.internal.message.Message;
import org.ros.message.MessageFactory;

import cav_msgs.ByteArray;
import cav_msgs.SPAT;
import gov.dot.fhwa.saxton.carma.rosutils.SaxtonLogger;

public class SPATMessage implements IMessage<SPAT> {
    
    protected SaxtonLogger log;
    protected MessageFactory messageFactory;
    
    public SPATMessage(MessageFactory factory, SaxtonLogger logger) {
        this.log            = logger;
        this.messageFactory = factory;
    }

    // Load libasn1c.so external C library
    static {
        try {
            System.loadLibrary("asn1c");
        } catch (Exception e) {
            System.out.println("Exception trapped while trying to load the asn1c library" + e.toString());
            e.printStackTrace();
        }
    }
    
    /**
     * This is the declaration for native method. It will take encoded SPAT byte array as input.
     * It will decode the message and set other byte array inputs values.
     * @return -1 means decode failed; 0 means decode is successful
     */
    public native int decodeSPAT(byte[] encodedArray, int[] intersectionData, int[][] movementStatesData);
    
    @Override
    public MessageContainer encode(Message plainMessage) {
        // This parser currently does not support encode SPAT messages 
        throw new UnsupportedOperationException();
    }

    @Override
    public MessageContainer decode(ByteArray binaryMessage) {
        // TODO Add actual JNI Call
        return null;
    }

}
