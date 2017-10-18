package gov.dot.fhwa.saxton.carma.guidance.maneuvers;

/**
 * Specifies the data input interface for a maneuver.  This keeps knowledge of the ROS network out of the remainder
 * of the package.
 */
public interface IManeuverInputs {

    /**
     * Provides the vehicle's current distance from the beginning of the planned route.
     * @return distance, m
     */
    double getDistanceFromRouteStart();

    /**
     * Provides the vehicle's current speed.
     * @return current speed, m/s
     */
    double getCurrentSpeed();

    /**
     * Provides the vehicle's expected dynamic response lag time.
     * @return lag time, sec
     */
    double getResponseLag();
}
