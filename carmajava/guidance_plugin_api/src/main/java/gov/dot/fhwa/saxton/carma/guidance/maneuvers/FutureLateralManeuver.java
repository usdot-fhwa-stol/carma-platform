package gov.dot.fhwa.saxton.carma.guidance.maneuvers;

import gov.dot.fhwa.saxton.carma.guidance.IGuidanceCommands;
import gov.dot.fhwa.saxton.carma.guidance.util.ILogger;
import gov.dot.fhwa.saxton.carma.guidance.util.LoggerManager;

import java.util.ArrayList;
import java.util.List;

/**
 * Base class for a compound maneuver that exists in the lateral dimension. A future maneuver is initially defined only in terms
 * of its starting and ending location and speeds.  It contains one or more simple maneuvers and acts as a place holder
 * that can be inserted into a trajectory while its constituent simple maneuvers are planned on a separate thread.
 * In an attempt to keep its
 * structure as simple as possible, a future maneuver doesn't have the ability to plan itself; its owner will need to
 * explicitly plan each constituent maneuver. Also, as these constituent maneuvers are added to the collection, they
 * must be added contiguously, front to back, so as not to leave any unfilled gaps (unlike planning
 * a Trajectory, which can have individual maneuvers placed anywhere at any time, with any amount of gap between).
 */
public class FutureLateralManeuver extends LateralManeuver {

    protected double                            startDist_;
    protected double                            endDist_;
    protected double                            startSpeed_;
    protected double                            endSpeed_;
    protected double                            maneuversEnd_;
    protected List<ISimpleManeuver>             mvrs_ = null;
    protected IManeuverInputs                   inputs_;
    protected int                               executingIdx_ = 0;
    protected ILogger                           log_;
    protected final double                      CONCATENATION_TOLERANCE = 0.001; // meter


    public FutureLateralManeuver(IManeuverInputs inputs, double startDist, double startSpeed, double endDist, double endSpeed) {
        super();
        inputs_ = inputs;
        startDist_ = startDist;
        startSpeed_ = startSpeed;
        endDist_ = endDist;
        endSpeed_ = endSpeed;
        maneuversEnd_ = startDist;
        mvrs_ = new ArrayList<>();
        log_ = LoggerManager.getLogger();
        log_.debug("MVR", "FutureLateralManeuver instantiated from " + startDist + " to " + endDist + " m");
    }


    //the next three methods exist only to satisfy the ISimpleManeuver interface, but are not used for this type of
    // maneuver collector, as each of its constituent maneuvers will be planned individually before being assembled.

    @Override
    public void plan(IManeuverInputs inputs, IGuidanceCommands commands, double startDist) throws IllegalStateException {
        throw new IllegalStateException("FutureLateralManeuver.plan is not implemented - should not be called.");
    }

    @Override
    public double planToTargetDistance(IManeuverInputs inputs, IGuidanceCommands commands, double startDist, double endDist)
                                        throws IllegalStateException {
        throw new IllegalStateException("FutureLateralManeuver.planToTargetDistance is not implemented - should not be called.");
    }

    @Override
    public boolean canPlan(IManeuverInputs inputs, double startDist, double endDist) throws IllegalStateException {
        throw new IllegalStateException("FutureLateralManeuver.canPlan is not implemented - should not be called.");
    }


    /**
     * Adds the given maneuver as the next maneuver in the sequence.
     * @param mvr - the maneuver to be added
     * @return the location of the end of the last existing maneuver, in meters downtrack of route beginning
     * @throws IllegalStateException if the given maneuver doesn't fit the space available
     */
    public double addManeuver(ISimpleManeuver mvr) throws IllegalStateException {

        if (!(mvr instanceof LateralManeuver)) {
            throw new IllegalStateException("Attempted to add " + mvr.getClass() + " as a constituent in FutureLateralManeuver.");
        }

        if (mvr.getEndDistance() > endDist_) {
            throw new IllegalStateException("Attempting to add a maneuver that extends past end of FutureLateralManeuver.");
        }

        if (Math.abs(mvr.getStartDistance() - maneuversEnd_) > CONCATENATION_TOLERANCE) {
            throw new IllegalStateException("Attempting to add a maneuver to a FutureLateralManeuver with a gap or overlap");
        }

        mvrs_.add(mvr);
        maneuversEnd_ = mvr.getEndDistance();
        log_.debug("MVR", "Added new concrete maneuver to FutureLateralManeuver from " +
                    mvr.getStartDistance() + " to " + maneuversEnd_);

        return maneuversEnd_;
    }


    /**
     * Adds a list of sequential maneuvers to the container. If any were previously defined they will be
     * overwritten in the process.
     * @param mvrs - the list of maneuvers to be added
     * @return the location of the end of the last existing maneuver, in meters downtrack of route beginning
     * @throws IllegalStateException if the list of maneuvers doesn't fit into the space available
     */
    public double addManeuvers(List<ISimpleManeuver> mvrs) throws IllegalStateException {

        mvrs_.clear();
        maneuversEnd_ = startDist_;

        for (ISimpleManeuver m : mvrs) {
            addManeuver(m);
        }

        return maneuversEnd_;
    }


    /**
     * Indicates if the constituent maneuvers have filled that dimension of the future maneuver's allocated space
     */
    public boolean isFull() {
        return maneuversEnd_ > endDist_ - CONCATENATION_TOLERANCE;
    }


    @Override
    public boolean executeTimeStep() throws IllegalStateException {
        double currentLoc = inputs_.getDistanceFromRouteStart();

        //if the vehicle has crossed the start location then
        if (currentLoc >= startDist_) {

            //if we are past the end distance of the whole future maneuver then
            if (currentLoc >= endDist_) {
                //bail out, indicating whole future maneuver is complete
                return true;
            }

            //if we are past the end distance of the current constituent maneuver then
            if (currentLoc > mvrs_.get(executingIdx_).getEndDistance()) {
                //increment the index and make sure we have another maneuver there
                if (++executingIdx_ >= mvrs_.size()) {
                    throw new IllegalStateException("Attempting to execute a non-existent maneuver in FutureLateralManeuver.");
                }
            }
            log_.debug("MVR", "Executing constituent maneuver " + executingIdx_ + " in FutureLateralManeuver");

            //execute the constituent maneuver (if location is beyond the end distance of the maneuver
            // it will throw an IllegalStateException
            mvrs_.get(executingIdx_).executeTimeStep();
        }

        return false; //indicates the future maneuver hasn't yet been completed
    }


    @Override
    public double getAxleAngleCmd() {
        //no-op since this class's executeTimeStep() doesn't call it - will be replaced by the constituent maneuver's implementation
        log_.warn("MVR", "\\\\ in FutureLateralManeuver.getAxleAngleCmd() - shouldn't be here! \\\\");
        return 0.0;
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
     * Returns the end distance of the last constituent maneuver, which may be less than the end distance of the container
     */
    public double getLastDistance() { return maneuversEnd_; }
}
