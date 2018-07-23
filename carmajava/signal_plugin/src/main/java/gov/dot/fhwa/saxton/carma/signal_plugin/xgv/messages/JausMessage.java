package gov.dot.fhwa.saxton.carma.signal_plugin.xgv.messages;

import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.utils.BitStreamPacker;
import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.utils.BitStreamUnpacker;
import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.utils.GlidepathApplicationContext;
import gov.dot.fhwa.saxton.carma.signal_plugin.dvi.IGlidepathAppConfig;

import java.util.Arrays;

/**
 * JausMessage base class for generic JAUS Messages
 * Parses the JAUS header data out of a byte array containing a formatted JAUS message
 *
 * To access JAUS message field data create a subclass extending this and parse the payload fields
 * in the subclass constructor after allowing the superconstructor to parse the header and generate
 * the payload array.
 */

public class JausMessage {

    public static enum JausCommandCode {
        HEARTBEAT (0x4202),
        QUERY_COMPONENT_STATUS (0x2002),
        REPORT_COMPONENT_STATUS (0x4002),
        QUERY_VELOCITY_STATE (0x2404),
        REPORT_VELOCITY_STATE (0x4404),
        QUERY_DISCRETE_DEVICES (0x2406),
        REPORT_DISCRETE_DEVICES (0x4406),
        SET_MOTION_PROFILE (0xE328),
        REQUEST_COMPONENT_CONTROL (0x000D),
        CONFIRM_COMPONENT_CONTROL (0x000F),
        RESUME (0x0004),
        STANDBY (0x0003),
        RELEASE_COMPONENT_CONTROL (0x000E),
        REPORT_COMPONENT_CONTROL (0x400D),
        QUERY_COMPONENT_CONTROL (0x200D),
        QUERY_COMPONENT_AUTHORITY (0x2001),
        SET_COMPONENT_AUTHORITY (0x0001);

        private int code;
        JausCommandCode(int i) {
            this.code = i;
        }

        public static JausCommandCode fromInt(int i) {
            for (JausMessage.JausCommandCode code : JausMessage.JausCommandCode.values()) {
                if (code.getCode() == i) {
                    return code;
                }
            }
            return null;
        }

        public int getCode() {
            return this.code;
        }
    }

    private int priorityLevel;
    private int acknak;
    private boolean serviceConnectionFlag;
    private boolean experimentalFlag;
    private int version;
    private int commandCode;
    private int recipientJAUSID;
    private int senderJAUSID;
    private int messagePayloadSize;
    private int dataControl;
    private int sequenceNumber;
    private byte[] payload;

    // Copy constructor
    public JausMessage(JausMessage copy) {
        priorityLevel = copy.getPriorityLevel();
        acknak = copy.getAcknak();
        serviceConnectionFlag = copy.isServiceConnectionFlag();
        experimentalFlag = copy.isExperimentalFlag();
        version = copy.getVersion();
        commandCode = copy.getCommandCode();
        recipientJAUSID = copy.getRecipientJAUSID();
        senderJAUSID = copy.getSenderJAUSID();
        messagePayloadSize = copy.getMessagePayloadSize();
        dataControl = copy.getDataControl();
        sequenceNumber = copy.getSequenceNumber();
        payload = copy.getPayload();
    }

    public JausMessage(boolean experimentalFlag, int recipientJAUSID, byte[] payload, int commandCode) {
        IGlidepathAppConfig config = GlidepathApplicationContext.getInstance().getAppConfig();
        this.experimentalFlag = experimentalFlag;

        // Convert the Component ID into the outbout XGV JAUS ID
        BitStreamPacker p = new BitStreamPacker(true);
        p.writeByte(config.getXgvInstanceId()); // Instance id
        p.writeByte(recipientJAUSID); // Component id
        p.writeByte(config.getXgvNodeId()); // Node id
        // DGF - not sure this is right, but i'm guessing because configured jaus ID > 1 byte?????
        // this was in my local xgv debug branch and it works
        p.writeByte(config.getXgvSubsystemId()); // Subsystem id
        BitStreamUnpacker u = new BitStreamUnpacker(p.getBytes(), true);
        this.recipientJAUSID = u.readInt();

        this.payload = payload;
        this.commandCode = commandCode;

        // Protocol Constants
        this.priorityLevel = 6;
        this.acknak = 0;
        this.serviceConnectionFlag = false;
        this.version = 0x02;
        this.dataControl = 0;

        // Inferred data
        this.senderJAUSID = config.getSoftwareJausId();
        this.messagePayloadSize = payload.length;
        this.sequenceNumber = 0;
    }

    public int getPriorityLevel() {
        return priorityLevel;
    }

    public int getAcknak() {
        return acknak;
    }

    public boolean isServiceConnectionFlag() {
        return serviceConnectionFlag;
    }

    public boolean isExperimentalFlag() {
        return experimentalFlag;
    }

    public int getVersion() {
        return version;
    }

    public int getCommandCode() {
        return commandCode;
    }

    public int getRecipientJAUSID() {
        return recipientJAUSID;
    }

    public int getSenderJAUSID() {
        return senderJAUSID;
    }

    public int getMessagePayloadSize() {
        return messagePayloadSize;
    }

    public int getDataControl() {
        return dataControl;
    }

    public int getSequenceNumber() {
        return sequenceNumber;
    }

    public byte[] getPayload() {
        return payload;
    }

    public void requestAck() {
        this.acknak = 1;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (!(o instanceof JausMessage)) return false;

        JausMessage that = (JausMessage) o;

        if (acknak != that.acknak) return false;
        if (commandCode != that.commandCode) return false;
        if (dataControl != that.dataControl) return false;
        if (experimentalFlag != that.experimentalFlag) return false;
        if (messagePayloadSize != that.messagePayloadSize) return false;
        if (priorityLevel != that.priorityLevel) return false;
        if (recipientJAUSID != that.recipientJAUSID) return false;
        if (senderJAUSID != that.senderJAUSID) return false;
        if (sequenceNumber != that.sequenceNumber) return false;
        if (serviceConnectionFlag != that.serviceConnectionFlag) return false;
        if (version != that.version) return false;
        if (!Arrays.equals(payload, that.payload)) return false;

        return true;
    }

    @Override
    public int hashCode() {
        int result = priorityLevel;
        result = 31 * result + acknak;
        result = 31 * result + (serviceConnectionFlag ? 1 : 0);
        result = 31 * result + (experimentalFlag ? 1 : 0);
        result = 31 * result + version;
        result = 31 * result + commandCode;
        result = 31 * result + recipientJAUSID;
        result = 31 * result + senderJAUSID;
        result = 31 * result + messagePayloadSize;
        result = 31 * result + dataControl;
        result = 31 * result + sequenceNumber;
        result = 31 * result + Arrays.hashCode(payload);
        return result;
    }

    public JausMessage(byte[] packet) {
        // Parse header fields
        BitStreamUnpacker u = new BitStreamUnpacker(packet, true);
        priorityLevel = u.readBits(4);
        acknak = u.readBits(2);
        serviceConnectionFlag = u.readBool();
        experimentalFlag = u.readBool();
        version = u.readBits(6);
        u.readBits(2); // Reserved
        commandCode = u.readShort();
        recipientJAUSID = u.readInt();
        senderJAUSID = u.readInt();
        messagePayloadSize = u.readBits(12);
        dataControl = u.readBits(4);
        sequenceNumber = u.readShort();
        payload = u.readByteArray(messagePayloadSize);
    }

    /**
     * Converts the JausMessage to a byte[] formatted for network transmission
     * @return byte[] ready to be sent to a network socket
     */
    public byte[] serialize() {
        BitStreamPacker p = new BitStreamPacker(true);
        p.writeBits(priorityLevel, 4);
        p.writeBits(acknak, 2);
        p.writeBool(serviceConnectionFlag);
        p.writeBool(experimentalFlag);
        p.writeBits(version, 6);
        p.writeBits(0, 2);
        p.writeShort(commandCode);
        p.writeInt(recipientJAUSID);
        p.writeInt(senderJAUSID);
        p.writeBits(messagePayloadSize, 12);
        p.writeBits(dataControl, 4);
        p.writeShort(sequenceNumber);
        p.writeByteArray(payload);

        return p.getBytes();
    }

    @Override
    public String toString() {
        return "JausMessage{" +
                "priorityLevel=" + priorityLevel +
                ", acknak=" + acknak +
                ", serviceConnectionFlag=" + serviceConnectionFlag +
                ", experimentalFlag=" + experimentalFlag +
                ", version=" + version +
                ", commandCode=" + commandCode +
                ", recipientJAUSID=" + recipientJAUSID +
                ", senderJAUSID=" + senderJAUSID +
                ", messagePayloadSize=" + messagePayloadSize +
                ", dataControl=" + dataControl +
                ", sequenceNumber=" + sequenceNumber +
                ", payload=" + Arrays.toString(payload) +
                '}';
    }
}
