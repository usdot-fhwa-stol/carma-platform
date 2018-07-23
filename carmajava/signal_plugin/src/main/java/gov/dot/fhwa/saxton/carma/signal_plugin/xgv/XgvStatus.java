package gov.dot.fhwa.saxton.carma.signal_plugin.xgv;

import gov.dot.fhwa.saxton.carma.signal_plugin.xgv.messages.ReportDiscreteDevicesMessage;
import gov.dot.fhwa.saxton.carma.signal_plugin.xgv.messages.ReportStatusMessage;

/**
 * Encapsulates a summary of the status of relevant XGV components and states.
 *
 * Constructed from a Report Component Status message from the Motion Profile Driver and a Report Error Codes message.
 */

public class XgvStatus {
    // TODO: Verify that these fields correspond to expected physical events
    private ReportStatusMessage.StatusCode statusCode;
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
    private boolean on;
    private boolean mainFuelOrEnergyOn;
    private boolean auxiliaryEnergyOrFuelOn;
    private boolean autoStartInProgress;
    private boolean autoShutdownInProgress;
    private boolean parkingBrakeSet;
    private boolean hornEngaged;
    private ReportDiscreteDevicesMessage.XgvGearState gear;

    /**
     * TODO: *************** Added to more easily support testing and creating a minimal mock object for testing ************
     */
    public XgvStatus(boolean on, boolean manualOverrideEngaged, boolean safeStopStopEngaged,ReportDiscreteDevicesMessage.XgvGearState gear)  {
        this.on = on;
        this.manualOverrideEngaged = manualOverrideEngaged;
        this.safeStopStopEngaged = safeStopStopEngaged;
        this.gear = gear;
        this.parkingBrakeSet = false;
        this.statusCode = ReportStatusMessage.StatusCode.READY;
    }

    public XgvStatus(ReportStatusMessage mpdState, ReportDiscreteDevicesMessage ddState) {
        statusCode = mpdState.getStatusCode();
        manualOverrideEngaged = mpdState.isManualOverrideEngaged();
        safeStopPauseEngaged = mpdState.isSafeStopPauseEngaged();
        safeStopStopEngaged = mpdState.isSafeStopStopEngaged();
        safeStopLinkStatus = mpdState.getSafeStopLinkStatus();
        externalSafeStopEngaged = mpdState.isExternalSafeStopEngaged();
        computerSteeringControlEngaged = mpdState.isComputerSteeringControlEngaged();
        computerSpeedControlEngaged = mpdState.isComputerSpeedControlEngaged();
        doorPauseEngaged = mpdState.isDoorPauseEngaged();
        errorPauseEngaged = mpdState.isErrorPauseEngaged();
        emergencyManualOverrideEngaged = mpdState.isEmergencyManualOverrideEngaged();
        steeringNeedsInitialization = mpdState.isSteeringNeedsInitialization();
        steeringInitializationNeedsUserInput = mpdState.isSteeringInitializationNeedsUserInput();
        on = ddState.isOn();
        mainFuelOrEnergyOn = ddState.isMainFuelOrEnergyOn();
        auxiliaryEnergyOrFuelOn = ddState.isAuxiliaryEnergyOrFuelOn();
        autoStartInProgress = ddState.isAutoStartInProgress();
        autoShutdownInProgress = ddState.isAutoShutdownInProgress();
        parkingBrakeSet = ddState.isParkingBrakeSet();
        hornEngaged = ddState.isHornEngaged();
        gear = ddState.getGear();
    }

    /**
     * Setter for manualOverrideEngaged
     *
     * @param bFlag
     */
    public void setManualOverrideEngaged(boolean bFlag)   {
        this.manualOverrideEngaged = bFlag;
    }

    public ReportStatusMessage.StatusCode getStatusCode() {
        return statusCode;
    }

    /** If manual override is engaged the XGV is still in the control loop of the vehicle, it's just forwarding all driver
     * input. All XGV modules/components are still powered on during manual override. This may occur when the vehicle
     * is in neutral, for example.
     * @return Whether the XGV is currently acting in pass-through mode.
     */
    public boolean isManualOverrideEngaged() {
        return manualOverrideEngaged;
    }

    /** Pause commands the vehicle to gracefully decelerate to a stop and apply the brakes to keep the vehicle in place.
     * @return Whether the vehicle is being commanded into a pause state by the SafeStop
     */
    public boolean isSafeStopPauseEngaged() {
        return safeStopPauseEngaged;
    }

    /**
     * SafeStop Stop is like pause except immediate and forceful.
     * @return Whether the vehicle is commanded to stop by the SafeStop
     */
    public boolean isSafeStopStopEngaged() {
        return safeStopStopEngaged;
    }

    /**
     * @return 0 if the SafeStop has no link, 1 if it is bypassed by the switch in center console, and 2 if link is good
     */
    public int getSafeStopLinkStatus() {
        return safeStopLinkStatus;
    }

    /**
     * The red buttons on the outside of the vehicle will cause the system to enter SafeStop stop mode if engaged. In
     * such an event the buttons must be reset before continuing vehicle operation.
     * @return True if SafeStop Stop is engaged and an external SafeStop button is depressed, false otherwise.
     */
    public boolean isExternalSafeStopEngaged() {
        return externalSafeStopEngaged;
    }

    /**
     * Returns the configuration parameter that determines whether or not the XGV can command lateral steering of the
     * vehicle.
     */
    public boolean isComputerSteeringControlEngaged() {
        return computerSteeringControlEngaged;
    }

    /**
     * Returns the configuration parameter that determines whether or not the XGV can command longitudinal motion of the
     * vehicle.
     */
    public boolean isComputerSpeedControlEngaged() {
        return computerSpeedControlEngaged;
    }

    /**
     * Returns true if the XGV is paused due to the door being open, false otherwise.
     */
    public boolean isDoorPauseEngaged() {
        return doorPauseEngaged;
    }

    /**
     * Returns true if the XGV is paused due to an error, false otherwise.
     */
    public boolean isErrorPauseEngaged() {
        return errorPauseEngaged;
    }

    /**
     * Emergency Manual Override takes the XGV out of the vehicle control loop by hardware switch. Some XGV modules
     * power down in this state and will not respond to commands/requests.
     * @return True if EMO is engaged, false otherwise.
     */
    public boolean isEmergencyManualOverrideEngaged() {
        return emergencyManualOverrideEngaged;
    }

    /**
     * The XGV needs the steering to be initialized by rotating the steering wheel to the leftmost locking point then
     * all the way to the right. Sometimes the XGV can automate this process.
     * @return True if steering needs to be initialized, false otherwise.
     */
    public boolean isSteeringNeedsInitialization() {
        return steeringNeedsInitialization;
    }

    /**
     * When the XGV cannot automate the process of steering initialization it must rely on the user to do so.
     * @return True if the XGV requires the user to manually initialize the steering wheel, false otherwise.
     */
    public boolean isSteeringInitializationNeedsUserInput() {
        return steeringInitializationNeedsUserInput;
    }

    /**
     * Corresponds to the state of the vehicle's power system.
     * @return True if the vehicle is powered on, false otherwise.
     */
    public boolean isOn() {
        return on;
    }


    /**
     * Corresponds to the state of the vehicle's engines, both the combustion and electric components.
     * @return True if either propulsion system is on, false otherwise.
     */
    public boolean isMainFuelOrEnergyOn() {
        return mainFuelOrEnergyOn;
    }

    /**
     * Unsure what this corrseponds to in this vehicle context. Advise seeking input from TORC before using this value.
     * @return Do not use.
     */
    public boolean isAuxiliaryEnergyOrFuelOn() {
        return auxiliaryEnergyOrFuelOn;
    }

    /**
     * Corresponds to the state of the vehicle's normal startup sequence at power on.
     * @return True if the vehicle is not yet ready to be controlled during its startup sequence, false otherwise.
     */
    public boolean isAutoStartInProgress() {
        return autoStartInProgress;
    }

    /**
     * AutoShutdown is the graceful way to power down the vehicle, instead of a forced engine kill.
     * @return True if the vehicle is powering down, false otherwise.
     */
    public boolean isAutoShutdownInProgress() {
        return autoShutdownInProgress;
    }

    /**
     * Parking Brake should be disengaged prior to vehicle operation as a startup condition for automated vehicle control.
     * @return True if parking brake is engaged, false otherwise.
     */
    public boolean isParkingBrakeSet() {
        return parkingBrakeSet;
    }

    /**
     * Corresponds to whether the horn is being activated at the point in time this message was generated.
     * @return True if horn is active, false otherwise.
     */
    public boolean isHornEngaged() {
        return hornEngaged;
    }

    /**
     * During automated control the state of the transmission is not necessarily reflective of the state of the shifter
     * lever. Prior to automated control however, this can be used to determine if the vehicle is in neutral and ready
     * to be controlled by the XGV.
     * @return An XgvGearState enum value corresponding to the state of the vehicle's transmission
     */
    public ReportDiscreteDevicesMessage.XgvGearState getGear() {
        return gear;
    }

    @Override
    public String toString() {
        return "XgvStatus{" +
                "statusCode=" + statusCode +
                ", manualOverrideEngaged=" + manualOverrideEngaged +
                ", safeStopPauseEngaged=" + safeStopPauseEngaged +
                ", safeStopStopEngaged=" + safeStopStopEngaged +
                ", safeStopLinkStatus=" + safeStopLinkStatus +
                ", externalSafeStopEngaged=" + externalSafeStopEngaged +
                ", computerSteeringControlEngaged=" + computerSteeringControlEngaged +
                ", computerSpeedControlEngaged=" + computerSpeedControlEngaged +
                ", doorPauseEngaged=" + doorPauseEngaged +
                ", errorPauseEngaged=" + errorPauseEngaged +
                ", emergencyManualOverrideEngaged=" + emergencyManualOverrideEngaged +
                ", steeringNeedsInitialization=" + steeringNeedsInitialization +
                ", steeringInitializationNeedsUserInput=" + steeringInitializationNeedsUserInput +
                ", on=" + on +
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
