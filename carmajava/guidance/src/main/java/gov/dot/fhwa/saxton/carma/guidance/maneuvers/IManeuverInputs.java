package gov.dot.fhwa.saxton.carma.guidance.maneuvers;

/**
 * Specifies the data input interface for a maneuver.
 */
public interface IManeuverInputs {

    /**
     * Provides the vehicle's current distance from the beginning of the planned route.
     * @return distance, m
     */
    double getDistanceFromRouteStart();

    /**
     * Provides a smoothed picture of the vehicle's current speed.
     * @return current speed, m/s
     */
    double getCurrentSpeed();
}
