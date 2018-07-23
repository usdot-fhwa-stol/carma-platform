package gov.dot.fhwa.saxton.carma.signal_plugin.xgv.messages;

import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.utils.BitStreamUnpacker;
import gov.dot.fhwa.saxton.carma.signal_plugin.xgv.messages.JausMessage;

/**
 * Extension of JausMessage to represent the ReportStatusMessage (0x4002)
 */
public class ReportStatusMessage extends JausMessage {
    public static enum StatusCode {
        INITIALIZE,
        READY,
        STANDBY,
        SHUTDOWN,
        FAILURE,
        EMERGENCY
    }

    public StatusCode getStatusCode() {
        return statusCode;
    }

    public boolean isManualOverrideEngaged() {
        return manualOverrideEngaged;
    }

    public boolean isSafeStopPauseEngaged() {
        return safeStopPauseEngaged;
    }

    public boolean isSafeStopStopEngaged() {
        return safeStopStopEngaged;
    }

    public int getSafeStopLinkStatus() {
        return safeStopLinkStatus;
    }

    public boolean isExternalSafeStopEngaged() {
        return externalSafeStopEngaged;
    }

    public boolean isComputerSteeringControlEngaged() {
        return computerSteeringControlEngaged;
    }

    public boolean isComputerSpeedControlEngaged() {
        return computerSpeedControlEngaged;
    }

    public boolean isDoorPauseEngaged() {
        return doorPauseEngaged;
    }

    public boolean isErrorPauseEngaged() {
        return errorPauseEngaged;
    }

    public boolean isEmergencyManualOverrideEngaged() {
        return emergencyManualOverrideEngaged;
    }

    public boolean isSteeringNeedsInitialization() {
        return steeringNeedsInitialization;
    }

    public boolean isSteeringInitializationNeedsUserInput() {
        return steeringInitializationNeedsUserInput;
    }

    // State variables
    private StatusCode statusCode;
    private boolean manualOverrideEngaged;
    private boolean safeStopPauseEngaged;
    private boolean safeStopStopEngaged;
    private int safeStopLinkStatus;
    private boolean externalSafeStopEngaged;
    private boolean computerSteeringControlEngaged;
    private boolean computerSpeedControlEngaged;
    private boolean doorPauseEngaged;
    private boolean errorPauseEngaged;
    private boolean emergencyManualOverrideEngaged;
    private boolean steeringNeedsInitialization;
    private boolean steeringInitializationNeedsUserInput;

    public ReportStatusMessage(byte[] packet) {
        super(packet);
        parsePayload();
    }

    public ReportStatusMessage(JausMessage message) {
        super(message);
        parsePayload();
    }

    private void parsePayload() {
        // Extract primary status code
        BitStreamUnpacker u = new BitStreamUnpacker(super.getPayload(), true);
        int status = u.readBits(4);
        statusCode = StatusCode.values()[status];

        // Discard unused data. Reserved for vendor specified data, unused by XGV.
        u.readBits(4);

        // Parse Secondary Status Code
        u.readShort(); // Reserved

        // Primitive Driver and Motion Profile Driver Status
        manualOverrideEngaged = u.readBool();
        safeStopPauseEngaged = u.readBool();
        safeStopStopEngaged = u.readBool();
        safeStopLinkStatus = u.readBits(2);
        externalSafeStopEngaged = u.readBool();
        computerSteeringControlEngaged = u.readBool();
        computerSpeedControlEngaged = u.readBool();
        doorPauseEngaged = u.readBool();
        errorPauseEngaged = u.readBool();
        emergencyManualOverrideEngaged = u.readBool();
        steeringNeedsInitialization = u.readBool();
        steeringInitializationNeedsUserInput = u.readBool();
    }

}
