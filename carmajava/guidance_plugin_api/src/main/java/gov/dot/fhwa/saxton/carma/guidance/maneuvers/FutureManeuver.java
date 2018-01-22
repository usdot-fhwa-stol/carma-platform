package gov.dot.fhwa.saxton.carma.guidance.maneuvers;

import gov.dot.fhwa.saxton.carma.guidance.IGuidanceCommands;

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
public class FutureManeuver implements ISimpleManeuver {

    protected double                            startDist_;
    protected double                            endDist_;
    protected double                            startSpeed_;
    protected double                            endSpeed_;
    protected double                            longEnd_;
    protected double                            latEnd_;
    protected List<ISimpleManeuver>             longMvrs_ = null;
    protected List<ISimpleManeuver>             latMvrs_ = null;
    protected IManeuverInputs                   inputs_;
    protected int                               executingLatIdx_ = 0;
    protected int                               executingLonIdx_ = 0;
    protected final double                      CONCATENATION_TOLERANCE = 0.001; // meeter


    public FutureManeuver(IManeuverInputs inputs, double startDist, double startSpeed, double endDist, double endSpeed) {
        inputs_ = inputs;
        startDist_ = startDist;
        startSpeed_ = startSpeed;
        endDist_ = endDist;
        endSpeed_ = endSpeed;
        longEnd_ = startDist;
        latEnd_ = startDist;

        longMvrs_ = new ArrayList<>();
        latMvrs_ = new ArrayList<>();
    }


    //the next three methods exist only to satisfy the ISimpleManeuver interface, but are not used for this type of
    // maneuver collector, as each of its constituent maneuvers will be planned individually before being assembled.

    @Override
    public void plan(IManeuverInputs inputs, IGuidanceCommands commands, double startDist) throws IllegalStateException {
        throw new IllegalStateException("FutureManeuver.plan is not implemented - should not be called.");
    }

    @Override
    public double planToTargetDistance(IManeuverInputs inputs, IGuidanceCommands commands, double startDist, double endDist)
                                        throws IllegalStateException {
        throw new IllegalStateException("FutureManeuver.planToTargetDistance is not implemented - should not be called.");
    }

    @Override
    public boolean canPlan(IManeuverInputs inputs, double startDist, double endDist) {
        throw new IllegalStateException("FutureManeuver.canPlan is not implemented - should not be called.");
    }


    /**
     * Adds the given maneuver as the next longitudinal maneuver in the sequence.
     * @param mvr - the maneuver to be added
     * @return the location of the end of the last existing longitudinal maneuver, in meters downtrack of route beginning
     * @throws IllegalStateException if the given maneuver is not derived from LongitudinalManeuver or doesn't fit the space available
     */
    public double  addLongitudinalManeuver(ISimpleManeuver mvr) throws IllegalStateException {

        if (!(mvr instanceof LongitudinalManeuver)) {
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
     * Adds a list of sequential longitudinal maneuvers to the container. If any were previously defined they will be
     * overwritten in the process.
     * @param mvrs - the list of maneuvers to be added
     * @return the location of the end of the last existing longitudinal maneuver, in meters downtrack of route beginning
     * @throws IllegalStateException if the list of maneuvers doesn't fit into the space available, or if any of them is
     *          not derived from LongitudinalManeuver
     */
    public double addLongitudinalManeuvers(List<ISimpleManeuver> mvrs) throws IllegalStateException {

        longMvrs_.clear();
        longEnd_ = startDist_;

        for (ISimpleManeuver m : mvrs) {
            addLongitudinalManeuver(m);
        }

        return longEnd_;
    }


    /**
     * Adds the given maneuver as the next lateral maneuver in the sequence.
     * @param mvr - the maneuver to be added
     * @return the location of the end of the last existing lateral maneuver, in meters downtrack of route beginning
     * @throws IllegalStateException if the given maneuver is not derived from LateralManeuver or doesn't fit the space available
     */
    public double  addLateralManeuver(ISimpleManeuver mvr) throws IllegalStateException {

        if (!(mvr instanceof LateralManeuver)) {
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
     * Adds a list of sequential lateral maneuvers to the container. If any were previously defined they will be
     * overwritten in the process.
     * @param mvrs - the list of maneuvers to be added
     * @return the location of the end of the last existing lateral maneuver, in meters downtrack of route beginning
     * @throws IllegalStateException if the list of maneuvers doesn't fit into the space available, or if any of them is
     *          not derived from LateralManeuver
     */
    public double addLateralManeuvers(List<ISimpleManeuver> mvrs) throws IllegalStateException {

        latMvrs_.clear();
        latEnd_ = startDist_;

        for (ISimpleManeuver m : mvrs) {
            addLateralManeuver(m);
        }

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


    public double getLastLongitudinalDistance() { return longEnd_; }

    public double getLastLateralDistance() { return latEnd_; }


    @Override
    public boolean executeTimeStep() throws IllegalStateException {
        double currentLoc = inputs_.getDistanceFromRouteStart();

        //if the vehicle has crossed the start location then
        if (currentLoc >= startDist_) {

            //if we are past the end distance of the whole future maneuver then
            if (currentLoc >= endDist_) {
                //bail out
                return true;
            }

            //execute the current maneuvers
            executeTimeStep(longMvrs_, executingLonIdx_);
            executeTimeStep(latMvrs_, executingLatIdx_);
        }

        return false;
    }


    @Override
    public double getStartDistance() {
        return startDist_;
    }


    @Override
    public double getEndDistance() {
        return endDist_;
    }


    /**
     * Executes a time step on the current maneuver in the given list of maneuvers. Increments its current index
     * if necessary to always point to the current maneuver.
     * @param mvrs - the list of maneuvers to be executed
     * @param index - current index into mvrs
     * @throws IllegalStateException if the vehicle's current location is beyond the end of the last maneuver in the list
     *
     * Assumes we are still within the bounds of the FutureManeuver container and that it is full to the end with
     * subordinate ISimpleManeuvers in both dimensions.
     */
    protected void executeTimeStep(List<ISimpleManeuver> mvrs, int index) throws IllegalStateException {
        double currentLoc = inputs_.getDistanceFromRouteStart();

        //if we are past the end distance of the current maneuver then
        if (currentLoc > mvrs.get(index).getEndDistance()) {
            //increment the index and make sure we have another maneuver there
            if (++index >= mvrs.size()) {
                throw new IllegalStateException("Attempting to execute a non-existent maneuver in FutureManeuver.");
            }
        }

        //execute the maneuver (if location is beyond the end distance of the maneuver, it will throw an IllegalStateException
        mvrs.get(index).executeTimeStep();
    }
}
