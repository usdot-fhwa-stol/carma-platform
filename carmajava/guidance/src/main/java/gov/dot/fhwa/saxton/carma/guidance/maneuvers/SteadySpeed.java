package gov.dot.fhwa.saxton.carma.guidance.maneuvers;

import gov.dot.fhwa.saxton.carma.guidance.IGuidanceCommands;

/**
 * Represents a steady-speed longitudinal maneuver.
 * NOTE:  even this maneuver requires a target speed to be specified. Since the vehicle's actual speed may never
 * be suitably close to the desired speed from the previous maneuver, we can't rely on simply continuing with the
 * current actual speed.
 */
public class SteadySpeed extends LongitudinalManeuver {

    /**
     * Since steady speed is intended to continue the current speed, the required distance to complete the maneuver
     * is zero.  Therefore, the end distance will be set to the start distance, and the caller will have the option
     * to set it to whatever larger distance is desired.
     */
    @Override
    public void plan(IManeuverInputs inputs, IGuidanceCommands commands, double startDist) throws IllegalStateException {
        super.plan(inputs, commands, startDist);

        //set end distance to start distance

    }


    /**
     * Used only for SteadySpeed maneuver, this allows the caller to set the length of the maneuver to whatever it
     * desired.
     * @param endDist - the desired end distance from the beginning of the planned route, m
     */
    public void overrideEndDistance(double endDist) {
        endDist_ = endDist;
    }


    @Override
    public void executeTimeStep() throws IllegalStateException {

        //if current location is not within the defined boundaries for this maneuver throw exception

        //set command to the target speed

        //invoke the ACC override

        //send the command to the vehicle
    }
}
