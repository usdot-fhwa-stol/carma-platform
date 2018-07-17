package gov.dot.fhwa.saxton.glidepath.asd;

/**
 * Interface for a concrete ASD message
 */
public interface IAsdMessage {

    /**
     * Interface method to convert the provided packet byte array into an ASD specific message
     *
     * @param buf
     * @return true if the message was successfully parsed
     */
    public boolean parse(byte[] buf);


    /**
     * Returns the ID of the intersections that this message applies to.
     * @return intersections ID
     */
    int getIntersectionId();


    /**
     * Returns the version number of the message.
     * @return version number
     */
    int getContentVersion();
}
