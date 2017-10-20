package gov.dot.fhwa.saxton.carma.guidance;

/**
 * Interface specifying how vehicle commands are to be sent to the object that forward them to the controller driver(s).
 */
public interface IGuidanceCommands {

    /**
     * Sets the desired speed and acceleration limit to be sent to the vehicle controller hardware.
     *
     * @param speed The speed to output
     * @param accel The maximum allowable acceleration in attaining and maintaining that speed
     */
    void setCommand(double speed, double accel);
}
