package gov.dot.fhwa.saxton.carma.guidance.maneuvers;

/**
 * Represents a compound maneuver in the lateral dimension.
 */
public class FutureLateralManeuver extends FutureManeuver {

    public FutureLateralManeuver(IManeuverInputs inputs, double startDist, double startSpeed, double endDist, double endSpeed) {
        super(inputs, startDist, startSpeed, endDist, endSpeed);
        log_.debug("MVR", "FutureLateralManeuver instantiated from " + startDist + " to " + endDist + " m");
    }


    /**
     * Adds the given maneuver as the next lateral maneuver in the sequence.
     * @param mvr - the maneuver to be added
     * @return the location of the end of the last existing longitudinal maneuver, in meters downtrack of route beginning
     * @throws IllegalStateException if the given maneuver is not derived from LateralManeuver or doesn't fit the space available
     */
    public double addManeuver(ISimpleManeuver mvr) throws IllegalStateException {

        if (!(mvr instanceof LateralManeuver)) {
            throw new IllegalStateException("Attempted to add " + mvr.getClass().getSimpleName() + " as a FutureLateralManeuver.");
        }
        log_.debug("MVR", "Adding new concrete maneuver to FutureLateralManeuver from " +
                    mvr.getStartDistance() + " to " + mvr.getEndDistance());

        return super.addManeuver(mvr);
    }
}
