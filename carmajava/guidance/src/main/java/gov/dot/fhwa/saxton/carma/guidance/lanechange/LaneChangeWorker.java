package gov.dot.fhwa.saxton.carma.guidance.lanechange;

/**
 * Provides all of the logic for planning a lane change tactical operation.
 */
public class LaneChangeWorker {

    /**
     * Plans the tactical operation (one or more maneuvers), which may include negotiating with neighbor vehicles.
     * @param startDist - starting location of the tactic, meters from beginning of route
     * @param endDist - ending location of the tactic, meters from beginning of route
     * @return true if a completed plan is immediately available; false if we need to wait on a negotiation
     */
    public boolean plan(double startDist, double endDist) {

        //verify that the input parameters have been defined already

        //create an empty container (future compound maneuver) for the TBD maneuvers to be inserted into

        //check for expected neighbor vehicles in our target area (target area is the target lane for the whole length
        // of the compound maneuver, since we could begin moving into that lane at any point along that length)

        //if no vehicles are expected to be in the way then
            //insert a simple lane change maneuver into the container

            //formulate an announcement for Negotiator to broadcast our intentions, just in case someone is there that
            // we don't know about (but we'll move out with our plan assuming we understand the environment correctly)

        //else (at least one vehicle has a chance of being in our way)
            //formulate a plan for coordinated maneuvering

    }


    public FutureManeuver getCompletedManeuver() {

    }


    public Plan getPlan() {

    }
}
