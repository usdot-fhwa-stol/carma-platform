package gov.dot.fhwa.saxton.glidepath.asd;

/**
 * Enum defining the type of Asd Message (MAP or SPAT)
 *
 * The type is the first byte within the raw packet that defines the raw data associated with the message type
 *
 * User: ferenced
 * Date: 1/19/15
 * Time: 9:24 AM
 *
 */
public enum AsdMessageType {
    MAP_MSG_ID (0x87),
    SPAT_MSG_ID (0x8D);

    private int type;
    AsdMessageType(int i) {
        this.type = i;
    }

    public int getType() {
        return this.type;
    }
}

