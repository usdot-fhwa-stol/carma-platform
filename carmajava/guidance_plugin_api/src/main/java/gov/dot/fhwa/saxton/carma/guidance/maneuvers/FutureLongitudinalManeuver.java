package gov.dot.fhwa.saxton.carma.guidance.maneuvers;

import java.util.List;

/**
 * Represents a compound maneuver in the longitudinal dimension.
 */
public class FutureLongitudinalManeuver extends FutureManeuver {

    public FutureLongitudinalManeuver(IManeuverInputs inputs, double startDist, double startSpeed, double endDist, double endSpeed) {
        super(inputs, startDist, startSpeed, endDist, endSpeed);
        log_.debug("MVR", "FutureLongitudinalManeuver instantiated from " + startDist + " to " + endDist + " m");
    }


    /**
     * Adds the given maneuver as the next longitudinal maneuver in the sequence.
     * @param mvr - the maneuver to be added
     * @return the location of the end of the last existing longitudinal maneuver, in meters downtrack of route beginning
     * @throws IllegalStateException if the given maneuver is not derived from LongitudinalManeuver or doesn't fit the space available
     */
    public double addManeuver(ISimpleManeuver mvr) throws IllegalStateException {

        if (!(mvr instanceof LongitudinalManeuver)) {
            throw new IllegalStateException("Attempted to add " + mvr.getClass() + " as a FutureLongitudinalManeuver.");
        }
        log_.debug("MVR", "Adding new concrete maneuver to FutureLongitudinalManeuver from " +
                    mvr.getStartDistance() + " to " + mvr.getEndDistance());

        return super.addManeuver(mvr);
    }
}
