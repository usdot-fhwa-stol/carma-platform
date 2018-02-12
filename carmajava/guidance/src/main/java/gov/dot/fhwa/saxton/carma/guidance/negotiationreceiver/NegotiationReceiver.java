/*
 * Copyright (C) 2017 LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

package gov.dot.fhwa.saxton.carma.guidance.negotiationreceiver;

import cav_msgs.NewPlan;
import cav_msgs.PlanStatus;
import cav_msgs.PlanType;
import gov.dot.fhwa.saxton.carma.guidance.ManeuverPlanner;
import gov.dot.fhwa.saxton.carma.guidance.arbitrator.TrajectoryPlanningResponse;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.IManeuverInputs;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.LongitudinalManeuver;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.SimpleManeuverFactory;
import gov.dot.fhwa.saxton.carma.guidance.params.ParameterSource;
import gov.dot.fhwa.saxton.carma.guidance.plugins.AbstractPlugin;
import gov.dot.fhwa.saxton.carma.guidance.plugins.IStrategicPlugin;
import gov.dot.fhwa.saxton.carma.guidance.plugins.PluginServiceLocator;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.IPublisher;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.ISubscriber;
import gov.dot.fhwa.saxton.carma.guidance.trajectory.Trajectory;

import java.util.*;

/**
 * Negotiation Receiver Plugin subscribes to NewPlan message for any new plans proposed by negotiator
 * and it all calls ArbitratorService to replan a trajectory such that it can insert its new planed
 * maneuvers into the new trajectory. For plans with different planIds, the plugin inserts new plans
 * into the trajectory according to the timing of receiving. For now, it will let arbitrator to replan
 * everytime when it received a new plan message.
 */
public class NegotiationReceiver extends AbstractPlugin implements IStrategicPlugin {

    protected static final long SLEEP_DURATION = 50;
    protected static final double TARGET_SPEED_EPSILON = 0.1;

    protected ISubscriber<NewPlan> planSub_;
    protected IPublisher<PlanStatus> statusPub_;
    protected SimpleManeuverFactory maneuverFactory_;
    protected ManeuverPlanner planner_;
    protected Map<String, List<LongitudinalManeuver>> planMap = new Hashtable<>(); //planId to a list of maneuvers
    protected Queue<String> replanQueue = new LinkedList<>(); //a task list for replan jobs indicated by planIds
    protected double maxAccel_ = 2.5;
    protected String currentPlanId = null; //the next planId for replan
    protected boolean includeAccelDist_ = true;
    protected double slowSpeedTime_ = 4.0;

    public NegotiationReceiver(PluginServiceLocator pluginServiceLocator) {
        super(pluginServiceLocator);
        version.setName("Negotiation Receiver Plugin");
        version.setMajorRevision(1);
        version.setIntermediateRevision(0);
        version.setMinorRevision(0);
    }

    @Override
    public void onInitialize() {
        ParameterSource params = pluginServiceLocator.getParameterSource();
        maxAccel_ = params.getDouble("~vehicle_acceleration_limit", 2.5);
        includeAccelDist_ = params.getBoolean("~include_accel_dist", true);
        slowSpeedTime_ = params.getDouble("~slow_speed_time", 4.0);
        maneuverFactory_ = new SimpleManeuverFactory();
        planner_ = pluginServiceLocator.getManeuverPlanner();
        planSub_ = pubSubService.getSubscriberForTopic("new_plan", NewPlan._TYPE);
        statusPub_ = pubSubService.getPublisherForTopic("plan_status", PlanStatus._TYPE);
        log.info("Negotiation Receiver plugin initialized");
    }

    @Override
    public void onResume() {
        planSub_.registerOnMessageCallback(this::onPlanReceived);
        setAvailability(true);
        log.info("Negotiation Receiver plugin resumed");
    }

    @Override
    public void loop() throws InterruptedException {
        if((!replanQueue.isEmpty()) && currentPlanId == null) {
            currentPlanId = replanQueue.poll();
            log.info("Loop found a new plan id " + currentPlanId + " . Calling arbitrator to replan.");
            // TODO before we start to replan we need to add some checks
            // start replan
            pluginServiceLocator.getArbitratorService().notifyTrajectoryFailure();
        } else {
            //We are in one of three situations:
            //  1.Waiting for replanning
            //  2.Replanning
            //  3.No new replans avaliable
            Thread.sleep(SLEEP_DURATION);
        }

    }

    @Override
    public void onSuspend() {
        planSub_.registerOnMessageCallback(this::noAction);
        setAvailability(false);
        log.info("Negotiation Receiver plugin suspended");
    }

    @Override
    public void onTerminate() {
        planSub_.registerOnMessageCallback(this::noAction);
        log.info("Negotiation Receiver plugin terminated");
    }

    @Override
    public TrajectoryPlanningResponse planTrajectory(Trajectory trajectory, double expectedEntrySpeed) {
        if(currentPlanId != null) {
            List<LongitudinalManeuver> maneuvers = planMap.get(currentPlanId);
            double endDist = 0;
            for (LongitudinalManeuver m : maneuvers) {
                endDist = Math.max(endDist, m.getEndDistance());
            }
            // check if we have enough space to insert maneuvers
            if (endDist > trajectory.getEndLocation()) {
                log.info("The endDist of plan " + currentPlanId + " is larger than trajectory endLocation " + trajectory.getEndLocation());
                TrajectoryPlanningResponse response = new TrajectoryPlanningResponse();
                response.requestLongerTrajectory(endDist);
                log.info("Requesting for a longer trajectory ending at " + endDist);
                return response;
            }
            for (LongitudinalManeuver m : maneuvers) {
                log.info(String.format("Insert a new maneuver {start=%.2f,end=%.2f,startSpeed=%.2f,endSpeed=%.2f}",
                        m.getStartDistance(), m.getEndDistance(), m.getStartSpeed(), m.getTargetSpeed()));
                trajectory.addManeuver(m);
            }
            // prevent from duplicate replan but keep the record of handled planId
            planMap.put(currentPlanId, new LinkedList<>());
            // publish plan status
            PlanStatus status = statusPub_.newMessage();
            status.getHeader().setFrameId("0");
            status.setPlanId(currentPlanId);
            status.setAccepted(true);
            statusPub_.publish(status);
            currentPlanId = null;
        }
        return new TrajectoryPlanningResponse();
    }

    private void onPlanReceived(NewPlan plan) {
        String id = plan.getPlanId();
        //TODO also need to consider the expiration time for a plan
        if(!planMap.containsKey(id)) {
            log.info("Received new plan with planId: " + id);
            // TODO this maneuvers list may need to include lateral maneuvers in the future
            List<LongitudinalManeuver> maneuvers = new LinkedList<>();
            String planInputs = plan.getInputs();
            if(plan.getType().getType() == PlanType.CHANGE_LANE_LEFT || plan.getType().getType() == PlanType.CHANGE_LANE_RIGHT) {
                String[] splitInput = planInputs.split(", ");
                double proposedLaneChangeStartDist = Double.parseDouble(splitInput[0].split(":")[1]);
                double proposedLaneChangeStartSpeed = Double.parseDouble(splitInput[1].split(":")[1]);

                // Assume the neighbor vehicle is travelling exactly beside us at the same speed when following mvrs begin.
                // We first need to slow down fairly quickly while the neighbor vehicle waits (continues to cruise).
                // Once the gap opens enough, then the neighbor vehicle changes lanes (assume already have
                // the right gap in front of him, since we presumably did) while we speed up again to his original
                // cruising speed to follow him. ACC will be a handy feature here!
                //
                // So that he can start his lane change immediately upon crossing the threshold of the new FutureLongitudinalManeuver space,
                // because we have already slowed to open the gap for him. For now, assume the gap is 1 sec, so we need
                // to double that (vehicle length is negligible if we have ACC working).  That means adding 1 sec over, say, 5
                // sec to make it smooth, or operating at 80% of our current speed for 5 sec.  To get to that lower speed,
                // we assume we can decel at 2 m/s^2. At 10 m/s the slowdown can be achieved in 1 sec. At 35 m/s we will take
                // 3.5 sec to reach the lower speed.  Since this decel time will be widening the gap somewhat, they can probably
                // live with constant speed for only 4 sec, we accel back to the beginning speed.
                IManeuverInputs mInputs = planner_.getManeuverInputs();
                double curSpeed = mInputs.getCurrentSpeed();
                double responseLag = mInputs.getResponseLag();
                double slowSpeed = 0.8*proposedLaneChangeStartSpeed;
                double initialLagDist = responseLag * curSpeed; //time to respond to slowdown cmd
                double distAtLowerSpeed = slowSpeed * slowSpeedTime_;
                //this will still work in the unlikely case that curSpeed < slowSpeed (shouldn't happen in TO 13 testing)
                double distToDecel = initialLagDist + 0.5*(curSpeed + slowSpeed) * 0.5*Math.abs(curSpeed - slowSpeed); //2nd 0.5 is because decel = 2 m/s^2
                double finalLagDist = responseLag * slowSpeed;
                double distToAccel = finalLagDist + 0.5*(slowSpeed + proposedLaneChangeStartSpeed)*(0.1*proposedLaneChangeStartSpeed);
                double totalDist = distToDecel + distAtLowerSpeed + (includeAccelDist_ ? distToAccel : 0.0);
                log.debug("V2V", "calculated slowSpeed = " + slowSpeed + ", distAtLowerSpeed = "
                            + distAtLowerSpeed + ", distToDecel = " + distToDecel + ", distToAccel = " + distToAccel
                            + ", totalDist = " + totalDist);

                double startLocation = proposedLaneChangeStartDist - totalDist;
                log.debug("V2V", "calculated startLocation = " + startLocation);
                double locAfterReplan = mInputs.getDistanceFromRouteStart() + 0.1*curSpeed; //assume new trajectory can begin executing in 100 ms
                if (startLocation < locAfterReplan) {
                    log.warn("V2V", "calculated - insufficient distance for us to slow down. They are "
                                + (locAfterReplan - startLocation) + " m late. Sending NACK.");
                    // reject this plan
                    // TODO now it rejects a plan forever, need allow back and forth in future
                    statusPub_.publish(this.buildPlanStatus(id, false));
                    planMap.put(id, new LinkedList<>());
                    return;
                }

                //build maneuvers for this vehicle
                double endSlowdown = startLocation + distToDecel;
                double endConstant = endSlowdown + distAtLowerSpeed;
                double endSpeedup = endConstant + distToAccel;
                log.debug("MVR", "onPlanReceived: endSlowdown = " + endSlowdown
                            + ", endConstant = " + endConstant + ", endSpeedup = " + endSpeedup);

                //create a list of maneuvers that we should execute
                List<String> maneuversString = new ArrayList<>();
                maneuversString.add(startLocation + ":" + endSlowdown + ":" + proposedLaneChangeStartSpeed + ":" + slowSpeed);
                maneuversString.add(endSlowdown + ":" + endConstant + ":" + slowSpeed + ":" + slowSpeed);
                maneuversString.add(endConstant + ":" + endSpeedup + ":" + slowSpeed + ":" + proposedLaneChangeStartSpeed);

                //attempt to plan each maneuver
                int index = 0;
                for(String m : maneuversString) {
                    String[] params = m.split(":");
                    //the params format is <startDistance>:<endDistance>:<startSpeed>:<endSpeed>
                    double[] paramsInDouble = {Double.parseDouble(params[0]), Double.parseDouble(params[1]),
                                               Double.parseDouble(params[2]), Double.parseDouble(params[3])};
                    LongitudinalManeuver maneuver = maneuverFactory_.createManeuver(paramsInDouble[2], paramsInDouble[3]);
                    maneuver.setSpeeds(paramsInDouble[2], paramsInDouble[3]);
                    maneuver.setMaxAccel(maxAccel_);
                    planner_.planManeuver(maneuver, paramsInDouble[0], paramsInDouble[1]);

                    // check the adjusted target speed to see if it can plan without huge adjustment
                    // if not it will show a negative status on PlanStatus message and ignore this planId in future
                    if(Math.abs(maneuver.getTargetSpeed() - paramsInDouble[3]) > TARGET_SPEED_EPSILON) {
                        log.warn("V2V", "Cannot plan the proposed maneuvers within accel_max limits. Failed maneuver #" + index
                                    + ". Sending Ack failure message.");
                        statusPub_.publish(this.buildPlanStatus(id, false));
                        planMap.put(id, new LinkedList<>());
                        return;
                    }
                    maneuvers.add(maneuver);
                    ++index;
                }
            }
            log.info("V2V", "Maneuvers defined - adding them to queue for replanning.");
            planMap.put(id, maneuvers);
            replanQueue.add(id);
        }
    }

    private void noAction(NewPlan plan) {
        //NO-OP callback for NewPlan when this plugin is suspended or terminated
    }

    // build a PlanStatus message with inputs
    private PlanStatus buildPlanStatus(String planId, boolean accepted) {
        PlanStatus status = statusPub_.newMessage();
        status.getHeader().setFrameId("0");
        status.setPlanId(planId);
        status.setAccepted(accepted);
        return status;
    }
    
}
