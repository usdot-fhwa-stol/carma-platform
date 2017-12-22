package gov.dot.fhwa.saxton.carma.factory;

import cav_msgs.ByteArray;

public interface IMessageFactory<T> {
    /**
     * This method will take a ROS message and encode it into a binary ByteArray in message container
     * @param plainMessage the message to be encoded
     * @return null means there is an error
     */
    public MessageContainer<ByteArray> encode(Object plainMessage);
    
    /**
     * This method will take a ROS byteArray and decode it to a plain message in message container
     * @param binaryMessage the message to be decoded
     * @return null means there is an error
     */
    public MessageContainer<T> decode(ByteArray binaryMessage);
}
