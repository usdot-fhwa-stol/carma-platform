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

import cav_msgs.Maneuver;
import cav_msgs.NewPlan;
import cav_msgs.PlanStatus;
import gov.dot.fhwa.saxton.carma.guidance.ManeuverPlanner;
import gov.dot.fhwa.saxton.carma.guidance.arbitrator.TrajectoryPlanningResponse;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.LongitudinalManeuver;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.SimpleManeuverFactory;
import gov.dot.fhwa.saxton.carma.guidance.plugins.AbstractPlugin;
import gov.dot.fhwa.saxton.carma.guidance.plugins.IStrategicPlugin;
import gov.dot.fhwa.saxton.carma.guidance.plugins.PluginServiceLocator;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.IPublisher;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.ISubscriber;
import gov.dot.fhwa.saxton.carma.guidance.trajectory.Trajectory;
import org.jboss.netty.buffer.ChannelBuffers;

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

    public NegotiationReceiver(PluginServiceLocator pluginServiceLocator) {
        super(pluginServiceLocator);
        version.setName("Negotiation Receiver Plugin");
        version.setMajorRevision(1);
        version.setIntermediateRevision(0);
        version.setMinorRevision(0);
    }

    @Override
    public void onInitialize() {
        maxAccel_ = pluginServiceLocator.getParameterSource().getDouble("~vehicle_acceleration_limit", 2.5);
        maneuverFactory_ = new SimpleManeuverFactory();
        planner_ = pluginServiceLocator.getManeuverPlanner();
        planSub_ = pubSubService.getSubscriberForTopic("new_plan", NewPlan._TYPE);
        statusPub_ = pubSubService.getPublisherForTopic("plan_status", PlanStatus._TYPE);
        log.info("Negotiation Receiver plugin initialized");
    }

    @Override
    public void onResume() {
        planSub_.registerOnMessageCallback(this::onPlanReceived);
        log.info("Negotiation Receiver plugin resumed");
    }

    @Override
    public void loop() throws InterruptedException {
        if((!replanQueue.isEmpty()) && currentPlanId == null) {
            currentPlanId = replanQueue.poll();
            log.info("Find a new plan id " + currentPlanId + " . Calling arbitrator to replan.");
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
            String[] planId = currentPlanId.split(" ");
            byte[] planIdInByte = new byte[planId.length];
            for(int i = 0; i < planId.length; i++) {
                planIdInByte[i] = Byte.parseByte(planId[i]);
            }
            status.setPlanId(ChannelBuffers.copiedBuffer(planIdInByte));
            status.setAccepted(true);
            statusPub_.publish(status);
            currentPlanId = null;
        }
        return new TrajectoryPlanningResponse();
    }

    private void onPlanReceived(NewPlan plan) {
        StringBuffer planId = new StringBuffer();
        for(int i = 0; i < plan.getPlanId().capacity(); i++) {
            planId.append(" " + plan.getPlanId().getByte(i));
        }
        String id = planId.substring(1);
        //TODO also need to consider the expiration time for a plan
        if(!planMap.containsKey(id)) {
            log.info("Received new plan with planId: " + id);
            // TODO this maneuvers list may need to include lateral maneuvers in the future
            List<LongitudinalManeuver> maneuvers = new LinkedList<>();
            for(Maneuver m : plan.getManeuvers()) {
                String[] params = m.getParameters().split(":");
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
                    log.warn("Cannot plan the proposed maneuvers within accel_max limits");
                    PlanStatus status = statusPub_.newMessage();
                    status.getHeader().setFrameId("0");
                    status.setPlanId(plan.getPlanId());
                    status.setAccepted(false);
                    statusPub_.publish(status);
                    planMap.put(id, new LinkedList<>());
                    return;
                }
                maneuvers.add(maneuver);
            }
            planMap.put(id, maneuvers);
            replanQueue.add(id);
        }
    }

    private void noAction(NewPlan plan) {
        //NO-OP callback for NewPlan when this plugin is suspended or terminated
    }

}
