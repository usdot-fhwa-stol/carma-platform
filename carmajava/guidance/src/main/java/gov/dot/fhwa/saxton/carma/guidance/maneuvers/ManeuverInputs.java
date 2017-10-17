package gov.dot.fhwa.saxton.carma.guidance.maneuvers;

/**
 * Interacts with the ROS network to provide the necessary input data to the Maneuver classes, allowing the rest
 * of the classes in this package to remain ignorant of ROS.
 */
public class ManeuverInputs implements IManeuverInputs {

    protected ISubscriber<Route>                 routeSubscriber_;
    protected ISubscriber<RoadwayEnvironment>    roadwayEnvSubscriber_;


    @Override
    public double getDistanceFromRouteStart() {

        //get the current distance from the Route message

        return 0.0; //TODO - bogus
    }


    @Override
    public double getCurrentSpeed() {

        //get the current vehicle speed from the Route message

        return 5.0; //TODO bogus
    }
}
