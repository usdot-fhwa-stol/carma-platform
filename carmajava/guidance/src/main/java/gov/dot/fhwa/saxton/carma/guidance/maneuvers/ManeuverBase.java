package gov.dot.fhwa.saxton.carma.guidance.maneuvers;

import gov.dot.fhwa.saxton.carma.guidance.IGuidanceCommands;

/**
 * Base class for all maneuver objects.
 */
public abstract class ManeuverBase implements IManeuver {

    protected double                            startDist_ = -1.0;
    protected double                            endDist_ = -1.0;
    protected IManeuverInputs                   inputs_;
    protected IGuidanceCommands                 commands_;


    /**
     * Provides the common planning capability that all maneuvers will need. Concrete maneuver classes
     * will need to provide their own plan() methods to fill in the details and execute this one first.
     */
    @Override
    public void plan(IManeuverInputs inputs, IGuidanceCommands commands, double startDist) throws IllegalStateException {
        inputs_ = inputs;
        commands_ = commands;
        startDist_ = startDist;
    }


    public abstract void executeTimeStep() throws IllegalStateException;


    public abstract void setSpeeds(double startSpeed, double targetSpeed) throws UnsupportedOperationException;


    public abstract void setTargetLane(int targetLane) throws UnsupportedOperationException;


    @Override
    public double getStartDistance() {
        return startDist_;
    }


    @Override
    public double getEndDistance() {
        return endDist_;
    }
}
