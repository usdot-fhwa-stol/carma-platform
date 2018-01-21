package gov.dot.fhwa.saxton.carma.guidance.lanechange;

import cav_msgs.MobilityAck;
import cav_msgs.MobilityIntro;
import cav_msgs.MobilityPlan;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.FutureManeuver;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.LaneChange;
import gov.dot.fhwa.saxton.carma.guidance.plugins.AbstractPlugin;
import gov.dot.fhwa.saxton.carma.guidance.plugins.ITacticalPlugin;
import gov.dot.fhwa.saxton.carma.guidance.plugins.PluginServiceLocator;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.IPublisher;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.ISubscriber;
import gov.dot.fhwa.saxton.carma.guidance.trajectory.Trajectory;
import org.ros.node.topic.Publisher;

import java.util.List;

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
    //
    //  - Assumes there is only one lane change being planned at a time. So the planSubtrajectory() method will not get
    //  called while the loop() method is actively working on current negotiations.

public class LaneChangePlugin extends AbstractPlugin implements ITacticalPlugin {

    private final int                       SLEEP_TIME = 50; //ms
    private int                             targetLane_ = -1;
    private double                          startSpeed_ = 0.0;
    private double                          endSpeed_ = 0.0;
    private MobilityIntro                   plan_ = null;           //TODO: type this as MobilityPlan once that type is implemented
    private ExtrapolatedEnvironment         env_ = new ExtrapolatedEnvironment();
    private IPublisher<MobilityIntro>       mobilityIntroPublisher_;
    private IPublisher<LaneChangeStatus>    laneChangeStatusPublisher_;
    private ISubscriber<MobilityAck>        mobilityAckSubscriber_;


    public LaneChangePlugin(PluginServiceLocator psl) {
        super(psl);
        version.setName("Lane Change Plugin");
        version.setMajorRevision(1);
        version.setIntermediateRevision(0);
        version.setMinorRevision(0);
    }


    @Override
    public void onInitialize() {

        //set up a publisher of mobility introduction messages
        mobilityIntroPublisher_ = pubSubService.getPublisherForTopic("~/mobility_intro", cav_msgs.MobilityIntro._TYPE);
        //set up publisher of status messages for the UI
        laneChangeStatusPublisher_ = pubSubService.getPublisherForTopic( "~/lane_change_status", cav_msgs.LaneChangeStatus._TYPE);

        //set up subscriber for mobility acks
        mobilityAckSubscriber_ = pubSubService.getSubscriberForTopic( "~/mobility_ack", cav_msgs.MobilityAck._TYPE);
    }


    @Override
    public void onResume() {
        //indicate always available

    }


    @Override
    public void loop() throws InterruptedException {

        //loop through all outstanding negotiations and process them
            //if this one indicates our plan was accepted then
                //add the new maneuver to the trajectory
                //remove the negotiation from the list of outstanding negotiations

        //sleep a while
    }


    @Override
    public void onSuspend() {
        //indicate not available - any in-progress negotiations at this time are subject to timing out
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
        targetLane_ = targetLane;
        startSpeed_ = startSpeed;
        endSpeed_ = endSpeed;
    }


    @Override
    public boolean planSubtrajectory(Trajectory traj, double startDistance, double endDistance) {
        boolean success = false;

        //verify that the input parameters have been defined already
        if (targetLane_ > -1) {

            //attempt to plan the lane change [plan]
            success = plan(startDistance, endDistance, targetLane_, startSpeed_, endSpeed_);

            //if it completed then
            if (success) {
                //insert the maneuver into the trajectory



            }
            //retrieve the plan announcement and give it to the Negotiator
            //add the plan to the list of in-work negotiations for monitoring
        }

        return success;
    }


    @Override
    public void onReceiveNegotiationRequest(String strategy) {
        //TODO - this will be the subject of a future work item
    }


    /**
     * Publishes the provided plan message onto the ROS network for the Negotiator to pick up
     */
    public void sendPlan() {





    }


    /**
     * Publishes a status message onto the ROS network for the UI to pick up
     */
    public void sendStatusUpdate(LaneChangeStatus msg) {

    }


    /**
     * Plans the tactical operation (one or more maneuvers), which may include negotiating with neighbor vehicles.
     * @param startDist - starting location of the tactic, meters from beginning of route
     * @param endDist - ending location of the tactic, meters from beginning of route
     * @return true if a completed plan is immediately available; false if we need to wait on a negotiation
     */
    private boolean plan(double startDist, double endDist, int targetLane, double startSpeed, double endSpeed) {

        //create an empty container (future compound maneuver) for the TBD maneuvers to be inserted into
        FutureManeuver future = new FutureManeuver(startDist, startSpeed, endDist, endSpeed);

        //check for expected neighbor vehicles in our target area (target area is the target lane for the whole length
        // of the compound maneuver, since we could begin moving into that lane at any point along that length)
        long futureTime = System.currentTimeMillis() + (long)(1000.0*2.0*(startDist - curDist)/(startSpeed + curSpeed));
        List<Object> vehicles = env_.getVehcilesInTargetArea(targetLane, startDist, endDist, futureTime);

        //if no vehicles are expected to be in the way then
        if (vehicles.size() == 0) {
            //insert a simple lane change maneuver into the container
            LaneChange lc = new LaneChange();
            try {
                lc.plan(inputs, commands, startDist);
            } catch (IllegalStateException e) {
                log.warn("V2V", "Exception when attempting to plan lane change: " + e.toString());
            }

            //formulate an announcement for Negotiator to broadcast our intentions, just in case someone is there that
            // we don't know about (but we'll move out with our plan assuming we understand the environment correctly)
            buildAnnouncement();
            return true;

        //else (at least one vehicle has a chance of being in our way)
        }else {
            //formulate a plan for coordinated maneuvering
            buildPlanMessage();
            return false;
        }
    }


    private void buildAnnouncement() {
        plan_ = mobilityIntroPublisher_.newMessage();








    }


    private void buildPlanMessage() {
        plan_ = mobilityIntroPublisher_.newMessage();








    }
}
