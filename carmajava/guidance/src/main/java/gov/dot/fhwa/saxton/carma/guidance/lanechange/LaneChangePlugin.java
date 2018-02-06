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
import gov.dot.fhwa.saxton.carma.guidance.pubsub.OnMessageCallback;
import gov.dot.fhwa.saxton.carma.guidance.trajectory.Trajectory;

import java.util.ArrayList;
import java.util.List;
import java.util.UUID;

/**
 * This is a mandatory plugin for the Carma platform that manages all lane change activity within a given sub-trajectory,
 * with the intent of changing from the current lane into one of the adjacent lanes only (no multi-lane hopping).
 *
 * It will create a FutureLongitudinalManeuver place holder for both lateral and longitudinal dimensions, which allows the parent
 * strategic plugin to continue planning the remainder of the trajectory around this space. In parallel to that activity,
 * this plugin will plan the details of the lane change, inserting one or more maneuvers into the FutureLongitudinalManeuver space
 * as necessary to accomplish the mission. This process will involve determining if neighbor vehicles are in the
 * desired target lane space, and, if so, negotiating a coordinated movement with one or more of them to ensure that
 * enough space is opened up in the target lane. Because this negotiation could take significant time, it will be
 * allowed to run in parallel with the planning of the rest of the trajectory (by the parent plugin), and, in fact,
 * can proceed even during execution of the early part of that trajectory, right up to the point that the contents of
 * this resultant FutureLongitudinalManeuver needs to be executed. At that time, if its planning is incomplete an exception will be
 * thrown and that trajectory will be aborted by the parent.
 */

    //TODO - this plugin was written in a very limited amount of time, so has made a lot of assumptions and simplifications.
    //       Below are some of the major things known now that should be refactored (not necessarily a complete list):
    //
    //  - As soon as negotiations fail, we should let arbitrator know so that it has a chance to abort & replace the
    //  current trajectory in an orderly way; otherwise the only way arbitrator will know is when the trajectory fails
    //  ugly because it attempts to execute an empty FutureLongitudinalManeuver.
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

    private final int                       EXPIRATION_TIME = 2000;
    private final int                       SLEEP_TIME = 50; //ms
    private int                             targetLane_ = -1;
    private double                          startSpeed_ = 0.0;
    private double                          endSpeed_ = 0.0;
    private MobilityIntro                   plan_ = null;           //TODO: type this as MobilityPlan once that type is implemented
    private List<Negotiation>               negotiations_ = new ArrayList<>();
    private ExtrapolatedEnvironment         env_ = new ExtrapolatedEnvironment();
    private FutureLongitudinalManeuver      futureLonMvr_ = null;
    private FutureLateralManeuver           futureLatMvr_ = null;
    private LaneChange                      laneChangeMvr_ = null;
    private IPublisher<MobilityIntro>       mobilityIntroPublisher_;
    private IPublisher<LaneChangeStatus>    laneChangeStatusPublisher_;
    private ISubscriber<MobilityAck>        mobilityAckSubscriber_;
    private final String                    STATIC_ID;


    public LaneChangePlugin(PluginServiceLocator psl) {
        super(psl);
        version.setName("Lane Change Plugin");
        version.setMajorRevision(1);
        version.setIntermediateRevision(0);
        version.setMinorRevision(0);
        STATIC_ID = UUID.randomUUID().toString();
    }


    @Override
    public void onInitialize() {

        //set up a publisher of mobility introduction & ack messages
        mobilityIntroPublisher_ = pubSubService.getPublisherForTopic("mobility_intro_outbound", cav_msgs.MobilityIntro._TYPE);
        
        //set up publisher of status messages for the UI
        laneChangeStatusPublisher_ = pubSubService.getPublisherForTopic( "~/lane_change_status", cav_msgs.LaneChangeStatus._TYPE);

        //set up subscriber for mobility acks
        mobilityAckSubscriber_ = pubSubService.getSubscriberForTopic( "mobility_ack_inbound", cav_msgs.MobilityAck._TYPE);
        mobilityAckSubscriber_.registerOnMessageCallback(new OnMessageCallback<MobilityAck>() {
            @Override
            public void onMessage(MobilityAck msg) {
                if (negotiations_.size() > 0) {
                    for (Negotiation n : negotiations_) {
                        n.newMessageArrived(msg);
                    }
                }else {
                    log.warn("V2V", "Received a MobilityAck message but no Negotiation objects exist! msg = " + msg.toString());
                }
            }
        });
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
            List<Negotiation> toBeRemoved = new ArrayList<>();
            for (Negotiation n : negotiations_) {
                LaneChangeStatus stat = laneChangeStatusPublisher_.newMessage();

                //if this one indicates our plan was accepted then
                if (n.process(stat)) {
                    //populate the future placeholder when no FutureManeuver is populated
                    if(!(futureLatMvr_.isFull() || futureLonMvr_.isFull())) {
                        populateFutureManeuver();
                    }
                    //remove the negotiation from the list of outstanding negotiations
                    toBeRemoved.add(n);
                }
            }
            // avoid current modification exceptions
            for(Negotiation n : toBeRemoved) {
                negotiations_.remove(n);
            }
        }

        //TODO: consider replacing this logic with a BlockingQueue to avoid doing all these gyrations & context switches

        //sleep a while
        Thread.sleep(SLEEP_TIME);
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


    public void setLaneChangeParameters(int targetLane, double startSpeed, double endSpeed) {
        targetLane_ = targetLane;
        startSpeed_ = startSpeed;
        endSpeed_ = endSpeed;
    }


    @Override
    public boolean planSubtrajectory(Trajectory traj, double startDistance, double endDistance) {
        log.info(String.format("In planSubtrajectory with params = {laneId=%d,startLimit=%.02f,endLimit=%.02f,start=%.02f,end=%.02f}",
            targetLane_,
            startSpeed_,
            endSpeed_,
            startDistance,
            endDistance));

        boolean planningComplete;

        //verify that the input parameters have been defined already
        if (targetLane_ > -1) {

            //attempt to plan the lane change [plan]
            try {
                planningComplete = plan(startDistance, endDistance, targetLane_, startSpeed_, endSpeed_);
                log.info("Plan returned " + planningComplete);
            }catch (IllegalStateException e) {
                //if we can't fit the maneuver in the space available, no point in starting negotiations to do so;
                // abort the whole subtrajectory, with no future maneuver inserted
                log.warn("Plan returned false!");
                return false;
            }

            //create empty containers (future compound maneuvers) for the TBD maneuvers to be inserted into
            ManeuverPlanner planner = pluginServiceLocator.getManeuverPlanner();
            IManeuverInputs inputs = planner.getManeuverInputs();
            futureLatMvr_ = new FutureLateralManeuver(this, inputs, startDistance, startSpeed_, endDistance, endSpeed_);
            futureLonMvr_ = new FutureLongitudinalManeuver(this, inputs, startDistance, startSpeed_, endDistance, endSpeed_);

            //insert these containers into the trajectory
            if (!traj.addManeuver(futureLatMvr_)  ||  !traj.addManeuver(futureLonMvr_)) {
                log.warn("Unable to add lane change future maneuvers to trajectory");
                return false;
            }

            //if it is safe to commit to the maneuver now, with no negotiations, then
            if (planningComplete) {
                //fill the future maneuver container with real stuff now
                populateFutureManeuver();
            }

            //add the plan to the list of in-work negotiations for monitoring (we can do this since both plan & announcement
            // occupy the same message type for the time being)
            Negotiation neg = new Negotiation(this);
            negotiations_.add(neg);

            //if we've reached this point, a future maneuver is at least possible, so parent can continue planning
            return true;

        }else {
            log.warn("V2V", "planSubtrajectory aborted because lane change parameters have not been defined.");
            return false;
        }
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
     * @param targetLane - ID of the lane we are heading toward
     * @param startSpeed - speed at the start of the tactical operation, m/s
     * @param endSpeed - speed at the end of the tactical operation, m/s
     * @return true if a completed plan is immediately available; false if we need to wait on a negotiation
     * @throws IllegalStateException if the lane change is geometrically infeasible
     */
    private boolean plan(double startDist, double endDist, int targetLane, double startSpeed, double endSpeed)
                        throws IllegalStateException {
        boolean planAvailable = false;
        ManeuverPlanner planner = pluginServiceLocator.getManeuverPlanner();
        IManeuverInputs inputs = planner.getManeuverInputs();
        double curDist = inputs.getDistanceFromRouteStart();
        double curSpeed = inputs.getCurrentSpeed();

        //check for expected neighbor vehicles in our target area (target area is the target lane for the whole length
        // of the compound maneuver, since we could begin moving into that lane at any point along that length)
        long futureTime = System.currentTimeMillis() + (long)(1000.0*2.0*(startDist - curDist)/(startSpeed + curSpeed));
        log.info("Expected arrival time at lane change area = " + futureTime);
        List<Object> vehicles = env_.getVehiclesInTargetArea(targetLane, startDist, endDist, futureTime);
        log.info("Num vehicles @ target area: " + vehicles.size());

        //construct our proposed simple lane change maneuver
        log.info("Creating lane change maneuver");
        laneChangeMvr_ = new LaneChange(this);
        laneChangeMvr_.setTargetLane(targetLane);
        if (planner.canPlan(laneChangeMvr_, startDist, endDist)) {
            log.info("Planning lane change maneuver...");
            planner.planManeuver(laneChangeMvr_, startDist, endDist);
            log.info("V2V", "plan: simple lane change maneuver is built.");
        }else {
            //TODO - would be nice to have some logic here to diagnose the problem and try again
            laneChangeMvr_ = null;
            log.warn("V2V", "plan: unable to construct the simple lane change maneuver.");
            throw new IllegalStateException("Proposed lane change maneuver won't fit the geometry.");
        }

        //if no vehicles are expected to be in the way then
        if (vehicles.size() == 0) {
            //insert the simple lane change maneuver into the container
            if (laneChangeMvr_ != null) {

                //formulate an announcement for Negotiator to broadcast our intentions, just in case someone is there that
                // we don't know about (but we'll move out with our plan assuming we understand the environment correctly)
                // TODO change it to an intro message when ExtrapolatedEnviroment logic is finished
                buildPlanMessage(inputs, targetLane, startDist, startSpeed);
                planAvailable = true;
            }
        }

        //if we can't do it outright then
        if (!planAvailable){
            // formulate a plan for coordinated maneuvering
            // we will not go into this case before ExtrapolatedEnviroment is ready
            buildPlanMessage(inputs, targetLane, startDist, startSpeed);
        }

        return planAvailable;
    }


    /**
     * Constructs a mobility Intro message intended to broadcast our intentions without any expectation of response.
     */
    private void buildAnnouncement(IManeuverInputs inputs, int targetLane) {
        plan_ = mobilityIntroPublisher_.newMessage();

        //TODO let the Negotiator fill in header info and other fields not defined here
        plan_.getHeader().setSenderId(this.STATIC_ID);
        plan_.getHeader().setRecipientId("00000000-0000-0000-0000-000000000000");
        plan_.getHeader().setPlanId(UUID.randomUUID().toString());
        plan_.getHeader().setTimestamp(System.currentTimeMillis());
        log.info("MobilityIntro has been built with planId" + plan_.getHeader().getPlanId());
        float speed = (float)inputs.getCurrentSpeed();
        byte lane = (byte)inputs.getCurrentLane();
        String link = "[Test track]"; //TODO - placeholder for testing
        short linkPos = (short)inputs.getDistanceFromRouteStart(); //TODO - for now assume a single link in the route
        long expiration = System.currentTimeMillis() + EXPIRATION_TIME; //TODO - not in use for now
        
        plan_.getMyEntityType().setType(BasicVehicleClass.DEFAULT_PASSENGER_VEHICLE); //TODO - move to negotiator node
        plan_.setForwardSpeed(speed);
        plan_.setMyLaneId(lane);
        plan_.setMyRoadwayLink(link);
        plan_.setMyRoadwayLinkPosition(linkPos);
        plan_.setExpiration(expiration);
        if (targetLane > inputs.getCurrentLane()) {
            plan_.getPlanType().setType(PlanType.CHANGE_LANE_LEFT);
        }else if (targetLane < inputs.getCurrentLane()) {
            plan_.getPlanType().setType(PlanType.CHANGE_LANE_RIGHT);
        }
    }


    /**
     * Constructs a detailed plan for coordinated movement that anticipates a response from a neighbor vehicle.
     *
     * For the current implementation we will largely use the same Intro message as for the announcement, (not enough
     * time to implement the full vision just yet).
     */
    private void buildPlanMessage(IManeuverInputs inputs, int targetLane, double startDist, double startSpeed) {
        //TODO - have this build a real plan message
        buildAnnouncement(inputs, targetLane);

        //use the capabilities string field to insert the specifics of what we will do
        String capabilities = String.format("[STARTDIST:%.1f, STARTSPEED:%.1f]", startDist, startSpeed);
        plan_.setCapabilities(capabilities);

    }

    /**
     * Fills up the future maneuver structure from beginning to end in both dimensions
     */
    private void populateFutureManeuver() {
        ManeuverPlanner planner = pluginServiceLocator.getManeuverPlanner();
        IManeuverInputs inputs = planner.getManeuverInputs();
        IGuidanceCommands commands = planner.getGuidanceCommands();

        try {
            //start the lane change immediately
            futureLatMvr_.addManeuver(laneChangeMvr_);

            //fill the remainder with a constant lane
            double startDist = futureLatMvr_.getLastDistance();
            double endDist = futureLatMvr_.getEndDistance();
            if (endDist - startDist > 0) {
                LaneKeeping lk = new LaneKeeping(this);
                lk.planToTargetDistance(inputs,commands, startDist, endDist);
                futureLatMvr_.addManeuver(lk);
            }

            //fill the whole longitudinal space with a constant speed
            SteadySpeed ss = new SteadySpeed(this);
            planner.planManeuver(ss, futureLonMvr_.getStartDistance(), endDist);
            futureLonMvr_.addManeuver(ss);

        }catch (IllegalStateException ise) {
            //log it to clarify the call sequence
            log.warn("V2V", "Exception trapped in populateFutureManeuver: " + ise.toString());
            throw ise;
        }
    }
}
