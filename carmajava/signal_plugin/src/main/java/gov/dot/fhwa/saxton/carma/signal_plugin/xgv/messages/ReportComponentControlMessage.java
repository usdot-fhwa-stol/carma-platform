package gov.dot.fhwa.saxton.carma.signal_plugin.xgv.messages;

import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.utils.BitStreamPacker;
import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.utils.BitStreamUnpacker;
import gov.dot.fhwa.saxton.carma.signal_plugin.xgv.messages.JausMessage;

/**
 * Parses and represents values for the JAUS Report Component Control Message (0x400D
 */
public class ReportComponentControlMessage extends JausMessage {
    private int currentControllerSubsystem;
    private int currentControllerNode;
    private int currentControllerComponent;
    private int currentControllerInstance;
    private int currentControllerAuthority;

    public ReportComponentControlMessage(JausMessage message) {
        super(message);
        parsePayload();
    }

    public ReportComponentControlMessage(byte[] packet) {
        super(packet);
        parsePayload();
    }

    private void parsePayload() {
        BitStreamUnpacker u = new BitStreamUnpacker(getPayload(), true);
        currentControllerSubsystem = u.readByte();
        currentControllerNode = u.readByte();
        currentControllerComponent = u.readByte();
        currentControllerInstance = u.readByte();
        currentControllerAuthority = u.readByte();
    }

    public int getCurrentControllerSubsystem() {
        return currentControllerSubsystem;
    }

    public int getCurrentControllerNode() {
        return currentControllerNode;
    }

    public int getCurrentControllerComponent() {
        return currentControllerComponent;
    }

    public int getCurrentControllerInstance() {
        return currentControllerInstance;
    }

    public int getCurrentControllerAuthority() {
        return currentControllerAuthority;
    }

    public int getCurrentControllerJausId() {
        BitStreamPacker p = new BitStreamPacker();
        p.writeByte(getCurrentControllerSubsystem());
        p.writeByte(getCurrentControllerNode());
        p.writeByte(getCurrentControllerComponent());
        p.writeByte(getCurrentControllerInstance());

        BitStreamUnpacker u = new BitStreamUnpacker(p.getBytes());
        return u.readInt();
    }
}
