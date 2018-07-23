package gov.dot.fhwa.saxton.carma.signal_plugin.xgv.messages;

import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.utils.BitStreamUnpacker;
import gov.dot.fhwa.saxton.carma.signal_plugin.xgv.messages.JausMessage;

/**
 * Parses and contains data about the states of the Discrete Devices on the XGV as reported by the primitive driver.
 */
public class ReportDiscreteDevicesMessage extends JausMessage {
    private int presenceVector;
    private boolean on;
    private boolean mainFuelOrEnergyOn;
    private boolean auxiliaryEnergyOrFuelOn;
    private boolean autoStartInProgress;
    private boolean autoShutdownInProgress;
    private boolean parkingBrakeSet;
    private boolean hornEngaged;
    private XgvGearState gear;

    public static enum XgvGearState {
        PARK,
        LOW,
        DRIVE,
        NEUTRAL,
        REVERSE
    }

    public ReportDiscreteDevicesMessage(JausMessage copy) {
        super(copy);
        parsePayload();
    }

    public ReportDiscreteDevicesMessage(byte[] packet) {
        super(packet);
        parsePayload();
    }

    private void parsePayload() {
        BitStreamUnpacker u = new BitStreamUnpacker(getPayload(), true);
        presenceVector = u.readByte();
        // TODO: Accomodate presence vector usage, for now we should only have one kind of message coming in

        // Main propulsion (1 byte)
        on = u.readBool();
        mainFuelOrEnergyOn = u.readBool();
        auxiliaryEnergyOrFuelOn = u.readBool();
        u.readBits(3); // Unused
        autoStartInProgress = u.readBool();
        autoShutdownInProgress = u.readBool();

        // Parking brake and horn (1 byte)
        parkingBrakeSet = u.readBool();
        hornEngaged = u.readBool();
        u.readBits(6); // Unused

        // Gear (1 byte)
        int gear = u.readByte();
        if (gear == 0) {
            this.gear = XgvGearState.PARK;
        } else if (gear == 1) {
            this.gear = XgvGearState.LOW;
        } else if (gear >= 2 && gear <= 127) {
            this.gear = XgvGearState.DRIVE;
        } else if (gear == 128) {
            this.gear = XgvGearState.NEUTRAL;
        } else if (gear >= 129 && gear <= 255) {
            this.gear = XgvGearState.REVERSE;
        }
    }

    public boolean isOn() {
        return on;
    }

    public boolean isMainFuelOrEnergyOn() {
        return mainFuelOrEnergyOn;
    }

    public boolean isAuxiliaryEnergyOrFuelOn() {
        return auxiliaryEnergyOrFuelOn;
    }

    public boolean isAutoStartInProgress() {
        return autoStartInProgress;
    }

    public boolean isAutoShutdownInProgress() {
        return autoShutdownInProgress;
    }

    public boolean isParkingBrakeSet() {
        return parkingBrakeSet;
    }

    public boolean isHornEngaged() {
        return hornEngaged;
    }

    public XgvGearState getGear() {
        return gear;
    }

    @Override
    public String toString() {
        return "ReportDiscreteDevicesMessage{" +
                "on=" + on +
                ", mainFuelOrEnergyOn=" + mainFuelOrEnergyOn +
                ", auxiliaryEnergyOrFuelOn=" + auxiliaryEnergyOrFuelOn +
                ", autoStartInProgress=" + autoStartInProgress +
                ", autoShutdownInProgress=" + autoShutdownInProgress +
                ", parkingBrakeSet=" + parkingBrakeSet +
                ", hornEngaged=" + hornEngaged +
                ", gear=" + gear +
                '}';
    }
}
