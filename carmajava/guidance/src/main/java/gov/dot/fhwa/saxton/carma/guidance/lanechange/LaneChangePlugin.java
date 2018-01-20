package gov.dot.fhwa.saxton.carma.guidance.lanechange;

import gov.dot.fhwa.saxton.carma.guidance.plugins.AbstractPlugin;
import gov.dot.fhwa.saxton.carma.guidance.plugins.ITacticalPlugin;
import gov.dot.fhwa.saxton.carma.guidance.plugins.PluginServiceLocator;
import gov.dot.fhwa.saxton.carma.guidance.trajectory.Trajectory;

/**
 * This is a mandatory plugin for the Carma platform that manages all lane change activity within a given sub-trajectory,
 * with the intent of changing from the current lane into one of the adjacent lanes only (no multi-lane hopping).
 *
 * It will create a FutureManeuver place holder for both lateral and longitudinal dimensions, which allows the parent
 * strategic plugin to continue planning the remainder of the trajectory around this space. In parallel to that activity,
 * this plugin will plan the details of the lane change, inserting one or more maneuvers into the FutureManeuver space
 * as necessary to accomplish the mission. This process will involve determining if neighbor vehicles are in the
 * desired target lane space, and, if so, negotiating a coordinated movement with one or more of them to ensure that
 * enough space is opened up in the target lane. Because this negotiation could take significant time, it will be
 * allowed to run in parallel with the planning of the rest of the trajectory (by the parent plugin), and, in fact,
 * can proceed even during execution of the early part of that trajectory, right up to the point that the contents of
 * this resultant FutureManeuver needs to be executed. At that time, if its planning is incomplete an exception will te
 * thrown and that trajectory will be aborted by the parent.
 */

    //TODO - this plugin was written in a very limited amount of time, so has made a lot of assumptions and simplifications.
    //       Below are some of the major things known now that should be refactored (not necessarily a complete list):
    //
    //  - As soon as negotiations fail, we should let arbitrator know so that it has a chance to abort & replace the
    //  current trajectory in an orderly way; otherwise the only way arbitrator will know is when the trajectory fails
    //  ugly because it attempts to execute an empty FutureManeuver.
    //
    //  - We have a very simplistic model for what our neighbor situation will look like at the time of maneuver
    //  execution. This could be beefed up in many ways.
    //
    //  - There are several complementary simplifications built into the Negotiator node as well.

public class LaneChangePlugin extends AbstractPlugin implements ITacticalPlugin {

    private final int               SLEEP_TIME = 100; //ms

    public LaneChangePlugin(PluginServiceLocator psl) {
        super(psl);
        version.setName("Lane Change Plugin");
        version.setMajorRevision(1);
        version.setIntermediateRevision(0);
        version.setMinorRevision(0);
    }


    @Override
    public void onInitialize() {
        //create the negotiation manager

        //set up a publisher of mpbility introduction messages
        //set up subscriber for mobility acks
        //set up publisher of status messages for the UI
        pubSubService.getSubscriberForTopic()
    }


    @Override
    public void onResume() {
        //indicate always available

    }


    @Override
    public void loop() throws InterruptedException {

        //loop through all outstanding negotiations and process them

        //sleep a while
    }


    @Override
    public void onSuspend() {
        //indicate not available - any in-progress negotiations at this time are subject to timing out

        //TODO - what to do about in-work negotiations?
    }


    @Override
    public void onTerminate() {
        //nothing to do
    }


    /**
     * Takes in the necessary inputs to define the boundary constraints for the lane change. This must be called prior
     * to calling planSubtrajectory.
     * Caution: it assumes that lane IDs will not change between the time it is called and the time that the maneuver
     * needs to be completed.
     * @param targetLane - ID of the lane we are moving into (could be to the right or left of current lane)
     * @param startSpeed - vehicle speed at beginning of the maneuver, m/s
     * @param endSpeed - vehicle speed at end of the maneuver, m/s
     */
    public void setLaneChangeParameters(int targetLane, double startSpeed, double endSpeed) {
        //store them


    }


    @Override
    public boolean planSubtrajectory(Trajectory traj, double startDistance, double endDistance) {

        //attempt to plan the lane change

        //if it completed then
            //insert the maneuver into the trajectory
        //retrieve the plan announcement and give it to the Negotiator
        //add the plan to the list of in-work negotiations for monitoring

        return false; //TODO - bogus
    }


    @Override
    public void onReceiveNegotiationRequest(String strategy) {
        //TODO - this will be the subject of a future work item
    }
}
