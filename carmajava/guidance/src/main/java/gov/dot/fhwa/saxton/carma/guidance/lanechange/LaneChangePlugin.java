package gov.dot.fhwa.saxton.carma.guidance.lanechange;

import cav_msgs.*;
import gov.dot.fhwa.saxton.carma.guidance.IGuidanceCommands;
import gov.dot.fhwa.saxton.carma.guidance.ManeuverPlanner;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.*;
import gov.dot.fhwa.saxton.carma.guidance.plugins.AbstractPlugin;
import gov.dot.fhwa.saxton.carma.guidance.plugins.ITacticalPlugin;
import gov.dot.fhwa.saxton.carma.guidance.plugins.PluginServiceLocator;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.IPublisher;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.ISubscriber;
import gov.dot.fhwa.saxton.carma.guidance.trajectory.Trajectory;
import gov.dot.fhwa.saxton.carma.rosutils.SaxtonLogger;
import org.ros.node.topic.Publisher;

import java.util.ArrayList;
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
    //
    //  - Lots of "TODO"s throughout this package

public class LaneChangePlugin extends AbstractPlugin implements ITacticalPlugin {

    private final int                       SLEEP_TIME = 50; //ms
    private int                             targetLane_ = -1;
    private double                          startSpeed_ = 0.0;
    private double                          endSpeed_ = 0.0;
    private MobilityIntro                   plan_ = null;           //TODO: type this as MobilityPlan once that type is implemented
    private List<Negotiation>               negotiations_ = new ArrayList<>();
    private ExtrapolatedEnvironment         env_ = new ExtrapolatedEnvironment();
    private FutureManeuver                  futureMvr_ = null;
    private LaneChange                      laneChangeMvr_ = null;
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
        setAvailability(true);
    }


    @Override
    public void loop() throws InterruptedException {

        //loop through all outstanding negotiations and process them
        if (negotiations_.size() > 0) {
            for (Negotiation n : negotiations_) {
                LaneChangeStatus stat = laneChangeStatusPublisher_.newMessage();

                //if this one indicates our plan was accepted then
                if (n.process(stat)) {
                    //populate the future placeholder
                    populateFutureManeuver();
                    //remove the negotiation from the list of outstanding negotiations
                    negotiations_.remove(n);
                }
            }
        }

        //sleep a while
        try {
            Thread.sleep(SLEEP_TIME);
        }catch (Exception e) {
        }
    }


    @Override
    public void onSuspend() {
        //indicate not available - any in-progress negotiations at this time are subject to timing out
        setAvailability(false);
    }


    @Override
    public void onTerminate() {
        //nothing to do
    }


    @Override
    public boolean planSubtrajectory(Trajectory traj, int targetLane, double startDistance, double startSpeed,
                                     double endDistance, double endSpeed) {
        boolean success = false;

        //verify that the input parameters have been defined already
        if (targetLane_ > -1) {

            //create an empty container (future compound maneuver) for the TBD maneuvers to be inserted into
            futureMvr_ = new FutureManeuver(startDistance, startSpeed, endDistance, endSpeed);
            //insert this container into the trajectory
            traj.addManeuver(futureMvr_);

            //attempt to plan the lane change [plan]
            success = plan(startDistance, endDistance, targetLane, startSpeed, endSpeed);

            //if it completed then
            if (success) {
                //fill the future maneuver container with real stuff now
                populateFutureManeuver();
            }

            //retrieve the plan announcement and give it to the Negotiator
            sendPlan();

            //add the plan to the list of in-work negotiations for monitoring
            Negotiation neg = new Negotiation(this, log);
            negotiations_.add(neg);
        }

        return success;
    }


    /**
     * Publishes the provided plan message onto the ROS network for the Negotiator to pick up
     */
    public void sendPlan() {
        if (plan_ != null) {
            mobilityIntroPublisher_.publish(plan_);
        }
    }


    /**
     * Publishes a status message onto the ROS network for the UI to pick up
     */
    public void sendStatusUpdate(LaneChangeStatus msg) {
        laneChangeStatusPublisher_.publish(msg);
    }


    /**
     * Plans the tactical operation (one or more maneuvers), which may include negotiating with neighbor vehicles.
     * @param startDist - starting location of the tactic, meters from beginning of route
     * @param endDist - ending location of the tactic, meters from beginning of route
     * @return true if a completed plan is immediately available; false if we need to wait on a negotiation
     */
    private boolean plan(double startDist, double endDist, int targetLane, double startSpeed, double endSpeed) {
        ManeuverPlanner planner = pluginServiceLocator.getManeuverPlanner();
        IManeuverInputs inputs = planner.getManeuverInputs();
        double curDist = inputs.getDistanceFromRouteStart();
        double curSpeed = inputs.getCurrentSpeed();
        int curLane = inputs.getCurrentLane();

        //check for expected neighbor vehicles in our target area (target area is the target lane for the whole length
        // of the compound maneuver, since we could begin moving into that lane at any point along that length)
        long futureTime = System.currentTimeMillis() + (long)(1000.0*2.0*(startDist - curDist)/(startSpeed + curSpeed));
        List<Object> vehicles = env_.getVehcilesInTargetArea(targetLane, startDist, endDist, futureTime);

        //if no vehicles are expected to be in the way then
        if (vehicles.size() == 0) {
            //insert a simple lane change maneuver into the container
            laneChangeMvr_ = new LaneChange();
            laneChangeMvr_.setTargetLane(targetLane);
            if (planner.canPlan(laneChangeMvr_, startDist, endDist)) {
                planner.planManeuver(laneChangeMvr_, startDist);

                //formulate an announcement for Negotiator to broadcast our intentions, just in case someone is there that
                // we don't know about (but we'll move out with our plan assuming we understand the environment correctly)
                buildAnnouncement(inputs, targetLane);
                return true;
            }
        }

        //formulate a plan for coordinated maneuvering
        buildPlanMessage(inputs, targetLane, startDist, startSpeed);
        return false;
    }


    /**
     * Constructs a mobility Intro message intended to broadcast our intentions without any expectation of response.
     */
    private void buildAnnouncement(IManeuverInputs inputs, int targetLane) {
        plan_ = mobilityIntroPublisher_.newMessage();

        //let the Negotiator fill in header info and other fields not defined here

        //we'll force the plan details into this string for now, but that's not what it's meant for
        //String capabilities;
        float speed = (float)startSpeed_;
        byte lane = (byte)targetLane;

        String link = "Test track"; //TODO - placeholder for testing
        short linkPos = (short)inputs.getDistanceFromRouteStart(); //TODO - for now assume a single link in the route

        plan_.setForwardSpeed(speed);
        plan_.setMyLaneId(lane);
        plan_.setMyRoadwayLink(link);
        plan_.setMyRoadwayLinkPosition(linkPos);
        if (targetLane > inputs.getCurrentLane()) {
            plan_.setPlanType(PlanType.CHANGE_LANE_LEFT);   //TODO - this is driving me nuts! Can't find a compatible type.
        }else if (targetLane < inputs.getCurrentLane()) {
            plan_.setPlanType(PlanType.CHANGE_LANE_RIGHT);
        }
    }


    /**
     * Constructs a detailed plan for coordinated movement that anticipates a response from a neighbor vehicle.
     *
     * For the current implementation we will largely use the same Intro message as for the announcement, but (not enough
     * time to implement the full vision just yet).
     */
    private void buildPlanMessage(IManeuverInputs inputs, int targetLane, double startDist, double startSpeed) {
        //TODO - have this build a real plan message
        buildAnnouncement(inputs, targetLane);

        //use the capabilities string field to insert the specifics of what we want the other vehicle to do
        // (a slowdown maneuver) -

        //Assume the neighbor vehicle is travelling exactly beside us at the same speed when this mvr begins.
        // They first need to slow down fairly quickly while we wait (continue to cruise).
        // Once the gap opens enough (no time to calculate that tonight), then we change lanes (assume already have
        // the right gap in front of us, since they presumably did) while they speed up again to their original
        // cruising speed to fall in behind us.  Their ACC will be a handy feature here!
        //
        // So that we can start our lane change immediately upon crossing the threshold of the new FutureManeuver space,
        // we need them to have already slowed to open the gap for us.  For now, assume the gap is 1 sec, so they need
        // to double that (vehicle length is negligible if we have ACC working).  That means adding 1 sec over, say, 5
        // sec to make it smooth, or operating at 80% of their current speed for 5 sec.  To get to that lower speed,
        // we assume they can decel at 2 m/s^2. At 10 m/s the slowdown can be achieved in 1 sec. At 35 m/s it will take
        // 3.5 sec to reach the lower speed.  Since this decel time will be widening the gap somewhat, we can probably
        // live with constant speed for only 4 sec, then accel back to the beginning speed.
        double distAtCurSpeed = inputs.getResponseLag() * startSpeed; //time to respond to slowdown cmd
        double distAtLowerSpeed = 0.8 * startSpeed * 4.0;
        double distToDecel = 0.9 * startSpeed * (0.1*startSpeed); //avg of start speed & the 80% speed for 1 sec for every 10 m/s
        double totalDist = 2.0*distToDecel + distAtCurSpeed + distAtLowerSpeed; //2x to account for accel at end
        log.debug("V2V", "buildPlanMessage distAtCurSpeed = " + distAtCurSpeed + ", distAtLowerSpeed = "
                    + distAtLowerSpeed + ", distToDecel = " + distToDecel + ", totalDist = " + totalDist);

        double startLocation = startDist - totalDist;
        log.debug("V2V", "buildPlanMessage startLocation = " + startLocation);
        if (startLocation < inputs.getDistanceFromRouteStart()) {
            log.warn("V2V", "buildPlanMessage - insufficient distance for other vehicle to slow down. We are "
                        + (inputs.getDistanceFromRouteStart() - startLocation) + " m late. Proceding anyway.");
        }

        double endSlowdown = startLocation + distToDecel;
        double slowSpeed = 0.8*startSpeed;
        double endConstant = endSlowdown + 4.0*slowSpeed;
        double endSpeedup = endSlowdown + distToDecel;

        String capabilities = String.format("SLOW:%.1f:%.1f:%.1f:%.1f CONST:%.1f:%.1f:%.1f:%.1f SPEEDUP:%.1f:%.1f:%.1f:%.1f",
                startLocation, endSlowdown, startSpeed, slowSpeed,
                endSlowdown, endConstant, slowSpeed, slowSpeed,
                endConstant, endSpeedup, slowSpeed, startSpeed);
        plan_.setCapabilities(capabilities);
    }

    /**
     * Fills up the future maneuver structure from beginning to end in both dimensions
     */
    public void populateFutureManeuver() {
        ManeuverPlanner planner = pluginServiceLocator.getManeuverPlanner();
        IManeuverInputs inputs = planner.getManeuverInputs();
        IGuidanceCommands commands = planner.getGuidanceCommands();

        try {
            //start the lane change immediately
            futureMvr_.addLateralManeuver(laneChangeMvr_);

            //fill the remainder with a constant lane
            LaneKeeping lk = new LaneKeeping();
            double startDist = futureMvr_.getLastLateralDistance();
            double endDist = futureMvr_.getEndDistance();
            lk.planToTargetDistance(inputs,commands, startDist, endDist);
            futureMvr_.addLateralManeuver(lk);

            //fill the whole longitudinal space with a constant speed
            SteadySpeed ss = new SteadySpeed();
            planner.planManeuver(ss, futureMvr_.getStartDistance(), endDist);
            futureMvr_.addLongitudinalManeuver(ss);
            
        }catch (IllegalStateException ise) {
            log.warn("V2V", "Exception trapped in populateFutureManeuver: " + ise.toString());
        }
    }
}
