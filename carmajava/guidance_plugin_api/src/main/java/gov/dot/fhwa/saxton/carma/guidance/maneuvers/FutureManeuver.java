package gov.dot.fhwa.saxton.carma.guidance.maneuvers;

import java.util.ArrayList;
import java.util.List;

/**
 * Represents a compound maneuver in both dimensions that is initially defined only in terms of its starting and ending
 * location and speeds.  It contains one or more simple maneuvers in each dimension. It acts as a place-holder in a
 * trajectory while its constituent simple maneuvers are planned on a separate thread.  In an attempt to keep its
 * structure as simple as possible, a FutureManeuver doesn't have the ability to plan itself; its owner will need to
 * explicitly plan each constituent maneuver. Also, as these constituent maneuvers are added to the collection, they
 * must be added contiguously, front to back, in each dimension, so as not to leave any unfilled gaps (unlike planning
 * a Trajectory, which can have individual maneuvers placed anywhere at any time, with any amount of gap between).
 */
public class FutureManeuver implements IManeuver {

    protected double                            startDist_;
    protected double                            endDist_;
    protected double                            startSpeed_;
    protected double                            endSpeed_;
    protected double                            longEnd_;
    protected double                            latEnd_;
    protected List<? extends ISimpleManeuver>   longMvrs_ = null;
    protected List<? extends ISimpleManeuver>   latMvrs_ = null;
    protected final double                      CONCATENATION_TOLERANCE = 0.1; // meeter


    public FutureManeuver(double startDist, double startSpeed, double endDist, double endSpeed) {
        startDist_ = startDist;
        startSpeed_ = startSpeed;
        endDist_ = endDist;
        endSpeed_ = endSpeed;
        longEnd_ = startDist;
        latEnd_ = startDist;

        longMvrs_ = new ArrayList<>();
        latMvrs_ = new ArrayList<>();
    }


    /**
     * Adds the given maneuver as the next longitudinal maneuver in the sequence.
     * @param mvr - the maneuver to be added
     * @return the location of the end of the last existing longitudinal maneuver, in meters downtrack of route beginning
     * @throws IllegalStateException if the given maneuver is not derived from LongitudinalManeuver or doesn't fit the space available
     */
    public double  addLongitudinalManeuver(ISimpleManeuver mvr) throws IllegalStateException {

        if (!(mvr extends LongitudinalManeuver)) {
            throw new IllegalStateException("Attempted to add " + mvr.getClass() + " as a longitudinal maneuver in FutureManeuver.");
        }

        if (mvr.getEndDistance() > endDist_) {
            throw new IllegalStateException("Attempting to add a longitudinal maneuver that extends past end of FutureManeuver.");
        }

        if (Math.abs(mvr.getStartDistance() - longEnd_) > CONCATENATION_TOLERANCE) {
            throw new IllegalStateException("Attempting to add a longitudinal maneuver to a Future Maneuver with a gap or overlap");
        }

        longMvrs_.add(mvr);
        longEnd_ = mvr.getEndDistance();

        return longEnd_;
    }


    /**
     * Adds the given maneuver as the next lateral maneuver in the sequence.
     * @param mvr - the maneuver to be added
     * @return the location of the end of the last existing lateral maneuver, in meters downtrack of route beginning
     * @throws IllegalStateException if the given maneuver is not derived from LateralManeuver or doesn't fit the space available
     */
    public double  addLateralManeuver(ISimpleManeuver mvr) throws IllegalStateException {

        if (!(mvr extends LateralManeuver)) {
            throw new IllegalStateException("Attempted to add " + mvr.getClass() + " as a lateral maneuver in FutureManeuver.");
        }

        if (mvr.getEndDistance() > endDist_) {
            throw new IllegalStateException("Attempting to add a lateral maneuver that extends past end of FutureManeuver.");
        }

        if (Math.abs(mvr.getStartDistance() - latEnd_) > CONCATENATION_TOLERANCE) {
            throw new IllegalStateException("Attempting to add a lateral maneuver to a Future Maneuver with a gap or overlap");
        }

        latMvrs_.add(mvr);
        latEnd_ = mvr.getEndDistance();

        return latEnd_;
    }


    /**
     * Indicates if the constituent longitudinal maneuvers have filled that dimension of the FutureManeuver's allocated space
     * @return true if yes
     */
    public boolean isFullLongitudinal() {
        return longEnd_ > endDist_ - CONCATENATION_TOLERANCE;
    }


    /**
     * Indicates if the constituend lateral maneuvers have filled that dimension of the FutureManeuver's allocated space
     * @return true if yes
     */
    public boolean isFullLateral() {
        return latEnd_ > endDist_ - CONCATENATION_TOLERANCE;
    }


    @Override
    public boolean executeTimeStep() throws IllegalStateException {









        return false; //TODO - bogus
    }


    @Override
    public double getStartDistance() {
        return startDist_;
    }


    @Override
    public double getEndDistance() {
        return endDist_;
    }
}
