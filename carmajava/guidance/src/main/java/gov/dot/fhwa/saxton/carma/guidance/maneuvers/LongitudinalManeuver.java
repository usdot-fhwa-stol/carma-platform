package gov.dot.fhwa.saxton.carma.guidance.maneuvers;

import gov.dot.fhwa.saxton.carma.guidance.IGuidanceCommands;

/**
 * Base class for all longitudinal maneuvers, providing the adaptive cruise control (ACC) functionality.
 */
public abstract class LongitudinalManeuver extends ManeuverBase {

    protected double                    startSpeed_ = -1.0; // m/s
    protected double                    endSpeed_ = -1.0;   // m/s
    protected double                    maxAccel_ = 0.999;     // m/s^2 absolute value; default is a conservative value
    protected final double              SMALL_SPEED_CHANGE = 2.5;   // m/s


    @Override
    public void plan(IManeuverInputs inputs, IGuidanceCommands commands, double startDist) throws IllegalStateException {
        super.plan(inputs, commands, startDist);

        //check that speeds have been defined
        if (startSpeed_ < 0.0  ||  endSpeed_ < 0.0) {
            throw new IllegalStateException("Longitudinal maneuver plan attempted without previously defining the start/target speeds.");
        }
    }


    public abstract boolean executeTimeStep() throws IllegalStateException;


    @Override
    public void setSpeeds(double startSpeed, double targetSpeed) throws UnsupportedOperationException {
        startSpeed_ = startSpeed;
        endSpeed_ = targetSpeed;
    }


    @Override
    public void setTargetLane(int targetLane) throws UnsupportedOperationException {
        throw new UnsupportedOperationException("Attempting to use setTargetLane on a longitudinal maneuver.");
    }


    /**
     * Specifies the maximum acceleration allowed in the maneuver.  Note that this value will apply to both speeding
     * up and slowing down (symmetrical).
     * @param limit - max (absolute value) allowed, m/s^2
     */
    public void setMaxAccel(double limit) {
        if (limit > 0.0) { //can't be equal to zero
            maxAccel_ = limit;
        }
    }


    /**
     * Implements the ACC capability to prevent the host vehicle from crashing into the vehicle in front of it.
     *
     * Each maneuver derived from this class needs to implement its executeTimeStep() method such that its last statement prior
     * to sending the speed command to the vehicle is a call to this method.
     * In order to maximize execution speed, there is no check provided to verify that
     * this sequence is implemented properly; it will be up to the author to manually ensure.
     *
     * @param rawCmd - the speed command computed assuming there is no preceding vehicle in the way, m/s
     * @return adjusted speed command that will prevent a crash with the preceding vehicle, m/s
     */
    protected double accOverride(double rawCmd) {

        //if we are too close to the preceding vehicle then
            //back off the commanded speed

        return rawCmd; //TODO bogus - implement in a future iteration
    }


    /**
     * Compares current following distance to configurable parameter for desired minimum time gap.
     * @return true if actual gap < specified minimum; false otherwise
     */
    private boolean tooCloseToFwdVehicle() {




        //TODO: implement this logic in a future iteration




        return false;
    }


    /**
     * Provides the ACC functionality of adjusting the speed command downward as necessary to regain the desired
     * minimum time gap.
     * @param rawCmd - the speed command computed assuming there is no preceding vehicle in the way, m/s
     * @return adjusted speed command that will prevent a crash with the preceding vehicle, m/s
     */
    private double adaptToFwdVehicle(double rawCmd) {



        //TODO:  implement this logic in a future iteration



        return rawCmd; //TODO bogus
    }
}
