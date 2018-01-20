package gov.dot.fhwa.saxton.carma.guidance.lanechange;

import cav_msgs.MobilityAck;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.IPubSubService;

import static gov.dot.fhwa.saxton.carma.guidance.lanechange.NegotiationMgr.State.COMPLETE;
import static gov.dot.fhwa.saxton.carma.guidance.lanechange.NegotiationMgr.State.NEW;
import static gov.dot.fhwa.saxton.carma.guidance.lanechange.NegotiationMgr.State.PLAN_SENT;


/**
 * Responsible for initiating negotiations with neighbor vehicles to work out a cooperative lane change tactic
 * and handling the results of that negotiation.  This only manages negotiations for a single plan, but it may
 * involve multiple neighbor vehicles.
 */
public class NegotiationMgr {

    enum State {
        NEW,
        PLAN_SENT,
        COMPLETE
    }

    enum StatusReport {
        //Caution: enum values must match those defined in cav_msgs/LaneChangeStatus.msg
        PLAN_SENT(1),
        ACCEPTANCE_RECEIVED(2),
        REJECTION_RECEIVED(3),
        OTHER_RECEIVED(4),
        PLANNING_SUCCESS(5),
        TIMED_OUT(6);

        StatusReport(int val) { val_ = val; }
        int val_;
    }


    public NegotiationMgr(FutureManeuver container, IPubSubService pubSubService) { //TODO also need List<vehicles>, plan

        //initialize our state to new


    }


    /**
     * Handles whatever processing is dictated by the current state and updated inputs from Negotiator.  This will be
     * called periodically from the plugin's loop() method.
     */
    public void process() {
        switch (state) {
            case NEW:
                //formulate an introduction message and send it to the negotiator
                //indicate that plan has been sent

            case PLAN_SENT:
                //if a new response has been received then
                    //if it is a rejection of the plan then
                        //inform other host nodes that our plan has been rejected
                        //notify arbitrator that our trajectory planning has failed [notifyTrajectoryFailure]
                        //indicate that we are complete

                    //else if plan was accepted then
                        //determine which vehicle it was from
                        //if it was from the vehicle of interest then
                            //inform other host nodes that our plan was accepted
                            //insert the concrete lane change maneuver into the container
                            //indicate we are complete

                    //else
                        //log the situation and ignore

            case COMPLETE:
                //log that this plan is concluded
                //return
        }

    }


    /**
     * Determines which of our neighbor vehicles sent the given ack message.
     * @param ack - the message in question
     * @return index of the sender in the list of known vehicles; -1 if unable to determine
     */
    private int determineSender(MobilityAck ack) {
        //loop through known vehicles
            //compute distance between its known location and location in the ack message
            //if it's the smallest so far, store it as candidate

        //if the candidate has a location error small enough then
            //indicate that we have found it
        //else
            //indicate we don't know where this message came from
            return -1;
    }

}
