package gov.dot.fhwa.saxton.carma.guidance.maneuvers;

import gov.dot.fhwa.saxton.carma.guidance.IGuidanceCommands;

/**
 * Defines an interface to all Maneuver objects.
 */
public interface IManeuver {

    /**
     * Plans the maneuver and makes it ready for execution
     *
     * @param inputs - the object that provides necessary input data about the route
     * @param commands - the object that will take output commands
     * @param startDist - distance from beginning of route at which this maneuver is to begin, m
     * @throws IllegalStateException if required target quantity is not defined prior to this call
     */
    void plan(IManeuverInputs inputs, IGuidanceCommands commands, double startDist) throws IllegalStateException;


    /**
     * Executes a single time step of the maneuver that has already been planned, by calculating the
     * instantaneous commands then passing those commands to the vehicle's controller driver.  There is no
     * assumption made about the duration or uniformity of time steps.
     *
     * ASSUMES that the plan method has already run to completion - there is no check for this condition!
     *
     * @return true if the maneuver has completed; false if it is still in progress
     * @throws IllegalStateException if called when vehicle's position along route is not between the
     * maneuver's start and end distances
     */
    boolean executeTimeStep() throws IllegalStateException;


    /**
     * Stores the beginning and target speed of the maneuver, to be used for longitudinal maneuvers only.
     * Since maneuvers will generally be chained together during planning, this is the only way that a maneuver
     * can know what speed the vehicle will have after completing its predecessor maneuver.
     * @param startSpeed - the expected speed at the beginning of the maneuver, m/s
     * @param targetSpeed - target speed at end of maneuver, m/s
     * @throws UnsupportedOperationException if called on a lateral maneuver object
     */
    void setSpeeds(double startSpeed, double targetSpeed) throws UnsupportedOperationException;


    /**
     * Stores the target lane ID, to be used for lateral maneuvers only.
     * @param targetLane - target lane number at end of maneuver
     * @throws UnsupportedOperationException if called on a longitudinal maneuver object
     */
    void setTargetLane(int targetLane) throws UnsupportedOperationException;


    /**
     * Retrieves the specified start distance of the maneuver.
     * @return distance from beginning of the route at which the maneuver begins, m
     */
    double getStartDistance();


    /**
     * Retrieves the calculated end distance of the maneuver.
     * @return distance from beginning of the route at which the maneuver is to complete, m
     */
    double getEndDistance();
}
