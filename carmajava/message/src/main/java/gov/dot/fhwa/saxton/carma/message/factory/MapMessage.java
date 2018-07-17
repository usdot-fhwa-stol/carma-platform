package gov.dot.fhwa.saxton.carma.message.factory;

import org.ros.internal.message.Message;
import org.ros.message.MessageFactory;

import cav_msgs.ByteArray;
import cav_msgs.MapData;
import gov.dot.fhwa.saxton.carma.rosutils.SaxtonLogger;

public class MapMessage implements IMessage<MapData>{

    protected SaxtonLogger log;
    protected MessageFactory messageFactory;
    
    public MapMessage(MessageFactory factory, SaxtonLogger logger) {
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
     * This is the declaration for native method. It will take encoded MAP byte array as input.
     * It will decode the message and set other byte array inputs values.
     * @param laneTypeData 
     * @return -1 means decode failed; 0 means decode is successful
     */
    public native int decodeMap(byte[] encodedArray, Object map_object, int[] intersectionData, int[] laneIDData,
            int[] ingressApproachData, int[] egressApproachData, int[] laneDirectionData, int[] laneTypeData, int[][] nodeOffsetData);
    
    @Override
    public MessageContainer encode(Message plainMessage) {
        // This class currently does not support encode MAP message 
        throw new UnsupportedOperationException();
    }

    @Override
    public MessageContainer decode(ByteArray binaryMessage) {
        // TODO add actual JNI function call
        return null;
    }

}
