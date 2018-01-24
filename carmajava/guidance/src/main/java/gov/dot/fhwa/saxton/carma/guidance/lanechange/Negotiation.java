package gov.dot.fhwa.saxton.carma.guidance.lanechange;

import cav_msgs.LaneChangeStatus;
import cav_msgs.MobilityAck;
import cav_msgs.MobilityAckType;
import gov.dot.fhwa.saxton.carma.guidance.util.ILogger;
import gov.dot.fhwa.saxton.carma.guidance.util.LoggerManager;
import gov.dot.fhwa.saxton.carma.rosutils.SaxtonLogger;

import java.util.ArrayList;
import java.util.List;

import static gov.dot.fhwa.saxton.carma.guidance.lanechange.Negotiation.State.COMPLETE;
import static gov.dot.fhwa.saxton.carma.guidance.lanechange.Negotiation.State.NEW;
import static gov.dot.fhwa.saxton.carma.guidance.lanechange.Negotiation.State.PLAN_SENT;


/**
 * Responsible for initiating negotiations with neighbor vehicles to work out a cooperative lane change tactic
 * and handling the results of that negotiation.  This only manages negotiations for a single plan, but it may
 * involve multiple neighbor vehicles.
 */
public class Negotiation {

    enum State {
        NEW,
        PLAN_SENT,
        COMPLETE
    }

    enum AckType {
        //Caution: these values are set to match those defined in MobilityAckType.msg
        UNKNOWN(0),
        ACCEPT_WITH_PROPOSAL_COMING(1),
        ACCEPT_WITH_EXECUTE(2),
        EXECUTION_COMPLETE(3),
        REJECT_WITH_COUNTERPROPOSAL_COMING(4),
        REJECT(5),
        REJECT_AND_TERMINATE_RELATIONSHIP(6),
        REJECT_WITH_SAFETY_CONCERN(7);

        AckType(int val) { val_ = val; }
        private int val_;
    }

    private State               state_;
    private List<MobilityAck>   newMessages_ = new ArrayList<>();
    private LaneChangePlugin    lcp_;
    private ILogger             log_;


    public Negotiation(LaneChangePlugin lcp) {
        lcp_ = lcp;
        state_ = NEW;
        log_ = LoggerManager.getLogger();
        log_.info("V2V", "New Negotiation object created in the LaneMergePlugin");
    }


    public synchronized void newMessageArrived(MobilityAck msg) {
        newMessages_.add(msg);
    }


    /**
     * Handles whatever processing is dictated by the current state and updated inputs from Negotiator.  This will be
     * called periodically from the plugin's loop() method.
     * @return true if the plan was accepted by the remote vehicle; false if it is rejected or timed out
     */
    public boolean process(LaneChangeStatus stat) {
        boolean success = false;

        switch (state_) {
            case NEW:
                //send the plan to the negotiator node
                lcp_.sendPlan();

                //indicate that plan has been sent
                stat.setStatus(LaneChangeStatus.PLAN_SENT);
                lcp_.sendStatusUpdate(stat);
                state_ = PLAN_SENT;
                log_.info("V2V", "New plan has been sent to Negotiator.");
                break;

            case PLAN_SENT:
                //if a new response has been received then
                if (newMessages_.size() > 0) {
                    log_.info("V2V", "Processing " + newMessages_.size() + " new Ack messages...");

                    synchronized(this) {
                        for (MobilityAck m : newMessages_) {

                            //if it is a rejection of the plan then
                            AckType ack = typeOfAck(m.getAgreement());
                            if(ack == AckType.REJECT_WITH_SAFETY_CONCERN  ||  ack == AckType.REJECT_AND_TERMINATE_RELATIONSHIP
                                    ||  ack == AckType.REJECT_WITH_COUNTERPROPOSAL_COMING  ||  ack == AckType.REJECT) {


                                //TODO: replace 0 with index of desired vehicle in as-yet-undefined list of vehicles
                                //if it was from the vehicle of interest then
                                if (determineSender(m) == 0) {

                                    //inform other host nodes that our plan has been rejected
                                    stat.setStatus(LaneChangeStatus.REJECTION_RECEIVED);
                                    lcp_.sendStatusUpdate(stat);

                                    //notify arbitrator that our trajectory planning has failed [notifyTrajectoryFailure]
                                    //TODO: future enhancement to avoid arbitrator having to do a panic replan when execution fails

                                    //indicate that we are complete
                                    state_ = COMPLETE;
                                    log_.warn("V2V", "Lane change plan has been rejected by remote vehicle.");
                                    break;
                                }

                            //else if plan was accepted then
                            }else if (ack == AckType.ACCEPT_WITH_EXECUTE) {

                                //TODO: replace 0 with index of desired vehicle in as-yet-undefined list of vehicles
                                //if it was from the vehicle of interest then
                                if (determineSender(m) == 0) {

                                    //inform other host nodes that our plan was accepted
                                    stat.setStatus(LaneChangeStatus.ACCEPTANCE_RECEIVED);
                                    lcp_.sendStatusUpdate(stat);

                                    //indicate we are complete
                                    success = true;
                                    state_ = COMPLETE;
                                    log_.info("V2V", "Lane change plan has been accepted by remote vehicle.");
                                    break; //break out of loop since don't care of others have rejected the plan
                                }

                            }else {
                                //log the situation and ignore
                                stat.setStatus(LaneChangeStatus.OTHER_RECEIVED);
                                lcp_.sendStatusUpdate(stat);
                                log_.info("V2V", "Received unactionable Ack of type " + ack.toString());
                            }

                        } //loop on new messages
                    } //synchronized
                    newMessages_.clear();
                } //if there are new messages
                break;

            case COMPLETE:
                //log that this plan is concluded
                log_.info("V2V", "Lane change planning is complete.");
                break;

            default:
                log_.warn("V2V", "NegotiatorMgr.process has detected an unknown state.");
        }

        //TODO - add logic to detect that the plan has timed out without a response

        return success;
    }


    private AckType typeOfAck(MobilityAckType mt) {
        switch (mt.getType()) {
            case MobilityAckType.ACCEPT_WITH_EXECUTE:
                return AckType.ACCEPT_WITH_EXECUTE;
            case MobilityAckType.ACCEPT_WITH_PROPOSAL_COMING:
                return AckType.ACCEPT_WITH_PROPOSAL_COMING;
            case MobilityAckType.EXECUTION_COMPLETE:
                return AckType.EXECUTION_COMPLETE;
            case MobilityAckType.REJECT:
                return AckType.REJECT;
            case MobilityAckType.REJECT_AND_TERMINATE_RELATIONSHIP:
                return AckType.REJECT_AND_TERMINATE_RELATIONSHIP;
            case MobilityAckType.REJECT_WITH_COUNTERPROPOSAL_COMING:
                return AckType.REJECT_WITH_COUNTERPROPOSAL_COMING;
            case MobilityAckType.REJECT_WITH_SAFETY_CONCERN:
                return AckType.REJECT_WITH_SAFETY_CONCERN;
            default:
                return AckType.UNKNOWN;
        }
    }


    /**
     * Determines which of our neighbor vehicles sent the given ack message.
     * @param ack - the message in question
     * @return index of the sender in the list of known vehicles; -1 if unable to determine
     */
    private int determineSender(MobilityAck ack) {
        //TODO - this will be fleshed out in a future work item

        //loop through known vehicles
            //compute distance between its known location and location in the ack message
            //if it's the smallest so far, store it as candidate

        //if the candidate has a location error small enough then
            //indicate that we have found it
        //else
            //indicate we don't know where this message came from

        return 0; //TODO - bogus but makes above code work for now
    }

}
