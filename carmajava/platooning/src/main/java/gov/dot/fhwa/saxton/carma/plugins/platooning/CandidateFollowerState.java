package gov.dot.fhwa.saxton.carma.plugins.platooning;

import java.util.UUID;

import cav_msgs.MobilityOperation;
import cav_msgs.MobilityRequest;
import cav_msgs.MobilityResponse;
import cav_msgs.PlanType;
import cav_msgs.SpeedAccel;
import gov.dot.fhwa.saxton.carma.guidance.arbitrator.TrajectoryPlanningResponse;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.LongitudinalManeuver;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.SimpleManeuverFactory;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.SteadySpeed;
import gov.dot.fhwa.saxton.carma.guidance.mobilityrouter.MobilityRequestResponse;
import gov.dot.fhwa.saxton.carma.guidance.plugins.PluginServiceLocator;
import gov.dot.fhwa.saxton.carma.guidance.trajectory.Trajectory;
import gov.dot.fhwa.saxton.carma.guidance.util.ILogger;
import gov.dot.fhwa.saxton.carma.guidance.util.RouteService;

/**
 * The CandidateFollowerState is a state which platooning algorithm is enabled on the current trajectory
 * anD the host vehicle is trying to change its current distance in order to join a front platoon.
 * This state will transit to Standby state when the algorithm is disabled when it is asked to plan;
 * it will transit to FollowerState if its PLATOON_CANDIDATE_JOIN request get an approval from the target
 * leader vehicle. It can also transit back to PlatoonLeader state if the current state is timeout or
 * the current PLATOON_CANDIDATE_JOIN request is rejected by the target platoon vehicle.
 * In this state, it will actively plan maneuvers in order to get closer with front platoon if necessary,
 * and it will monitor the maneuvers it is planned and see if the join process is interrupted. As a candidate
 * follower, it will not handle any JOIN request, not actively look for any other platoons and not broadcast
 * heart-beat but it will continue sending mobility STATUS if it has some followers.
 */
public class CandidateFollowerState implements IPlatooningState {

    protected static double LONGER_TRAJ_BUFFER_FACTOR = 1.5;
    
    protected PlatooningPlugin     plugin;
    protected ILogger              log;
    protected PluginServiceLocator pluginServiceLocator;
    protected double               speedUpTime;
    protected String               targetLeaderId;
    protected String               targetPlatoonId;
    private   PlatoonPlan          currentPlan;
    private   boolean              hasPlannedManeuvers;
    private   long                 stateStartTime;
    private   double               speedUpEndDistance;

    public CandidateFollowerState(PlatooningPlugin plugin, ILogger log, PluginServiceLocator pluginServiceLocator, double speedUpTime, String targetId, String newPlatoonId) {
        this.plugin               = plugin;
        this.log                  = log;
        this.pluginServiceLocator = pluginServiceLocator;
        this.speedUpTime          = speedUpTime;
        this.targetLeaderId       = targetId;
        this.targetPlatoonId      = newPlatoonId;
        this.hasPlannedManeuvers  = false;
        this.speedUpEndDistance   = Double.MAX_VALUE;
        this.stateStartTime       = System.currentTimeMillis();
    }

    @Override
    public TrajectoryPlanningResponse planTrajectory(Trajectory traj, double expectedEntrySpeed) {
        RouteService rs = pluginServiceLocator.getRouteService();
        TrajectoryPlanningResponse tpr = new TrajectoryPlanningResponse();
        // If there is no platooning window, we set plugin to Standby state
        if(!rs.isAlgorithmEnabledInRange(traj.getStartLocation(), traj.getEndLocation(), plugin.PLATOONING_FLAG)) {
            log.debug("There is no platoon window in " + traj.toString() + " . Change back to standby state");
            // TODO Send out mobility request message to inform its LEAVE and delegate the leader job properly
            plugin.setState(new StandbyState(plugin, log, pluginServiceLocator));
            // If we have already planned a speed up trajectory, it will be interrupted 
            if(hasPlannedManeuvers) {
                // TODO Maybe need to inform the front waiting leader to abort current plan and stop waiting
                log.warn("We have planned a speed up but the plan is interrupted due to no avaliable plan window");
            }
        } else {
            if(hasPlannedManeuvers) {
                // If we are in this block, it means the current plan is interrupted, we change back to normal leader state
                // TODO Maybe need to inform the front waiting leader to abort current plan and stop waiting
                log.debug("We have planned a speed up but the plan is interrupted due to a re-plan event");
                log.debug("Change back to PlatoonLeaderState and try to join another platoon");
                plugin.setState(new PlatoonLeaderState(plugin, log, pluginServiceLocator));
            } else {
                // Plan a speed up maneuver to speed limit and a steady speed maneuver at the speed limit if necessary
                if(speedUpTime > 0) {
                    SimpleManeuverFactory maneuverFactory = new SimpleManeuverFactory(plugin);
                    double currentSpeed = pluginServiceLocator.getManeuverPlanner().getManeuverInputs().getCurrentSpeed();
                    double speedLimit = rs.getSpeedLimitAtLocation(rs.getCurrentDowntrackDistance()).getLimit();
                    LongitudinalManeuver speedUp = maneuverFactory.createManeuver(currentSpeed, speedLimit);
                    speedUp.setSpeeds(currentSpeed, speedLimit);
                    speedUp.setMaxAccel(plugin.getMaxAccel());
                    pluginServiceLocator.getManeuverPlanner().planManeuver(speedUp, traj.getStartLocation());
                    double speedUpEndDtd = speedUp.getEndDistance();
                    log.debug("Planned a speedUp maneuver " + speedUp.toString());
                    // Calculate how far the host vehicle should stay on the speed limit based on the required time
                    this.speedUpEndDistance = speedUpEndDtd + (speedLimit * this.speedUpTime);
                    log.debug("If we plan a steady speed maneuver to catch up the gap, we need trajectory to end at " + this.speedUpEndDistance);
                    if(this.speedUpEndDistance > traj.getEndLocation()) {
                        log.debug("steadySpeedManeuverEnd exceeds the end of current trajectory, requesting a longer one.");
                        double totalDistanceNeeded = LONGER_TRAJ_BUFFER_FACTOR * (this.speedUpEndDistance - traj.getStartLocation());
                        log.debug("Request to extend the trajectory to " + traj.getStartLocation() + totalDistanceNeeded);
                        tpr.requestLongerTrajectory(traj.getStartLocation() + totalDistanceNeeded);
                    } else {
                        LongitudinalManeuver steadySpeed = maneuverFactory.createManeuver(speedLimit, speedLimit);
                        steadySpeed.setSpeeds(speedLimit, speedLimit);
                        steadySpeed.setMaxAccel(plugin.getMaxAccel());
                        pluginServiceLocator.getManeuverPlanner().planManeuver(steadySpeed, speedUpEndDtd);
                        ((SteadySpeed) steadySpeed).overrideEndDistance(this.speedUpEndDistance);
                        log.debug("Planned steady speed maneuver " + steadySpeed.toString());
                        boolean isSpeedUpInserted = traj.addManeuver(speedUp);
                        boolean isSteadySpeedInserted = traj.addManeuver(steadySpeed);
                        if(isSpeedUpInserted && isSteadySpeedInserted) {
                            log.debug("Inserted a speed up and a steady speed maneuver to close the gap.");
                            this.hasPlannedManeuvers = true;
                        } else {
                            log.debug("Insertion of speed up maneuver and steady speed maneuver fails:");
                            log.debug("isSpeedUpInserted = " + isSpeedUpInserted + ", isSteadySpeedInserted = " + isSteadySpeedInserted);
                            log.debug("Assuming the space has been used in the trajectory and requesting a higher priority");
                            tpr.requestHigherPriority();
                        }
                    }
                } else {
                    log.debug("We do not need to speed up to join the front platoon because we are already close enough");
                    this.hasPlannedManeuvers = true;
                }
            }
        }
        return tpr;
    }

    @Override
    public MobilityRequestResponse onMobilityRequestMessgae(MobilityRequest msg) {
        // This state does not handle any mobility request for now
        // TODO Maybe it should handle some ABORT request from a waiting leader
        log.debug("Recived mobility request with type " + msg.getPlanType().getType() + " but ignored.");
        return MobilityRequestResponse.NO_RESPONSE;
    }

    @Override
    public void onMobilityOperationMessage(MobilityOperation msg) {
        // We still need to handle STATUS operAtion message from our platoon
        String strategyParams = msg.getStrategyParams();
        boolean isPlatoonStatusMsg = strategyParams.startsWith("STATUS");
        if(isPlatoonStatusMsg) {
            String vehicleID = msg.getHeader().getSenderId();
            String platoonId = msg.getHeader().getPlanId();
            String statusParams = strategyParams.split("|")[1];
            plugin.getPlatoonManager().memberUpdates(vehicleID, platoonId, statusParams);
            log.debug("Received platoon status message from " + msg.getHeader().getSenderId());
        } else {
            log.debug("Received a mobility operation message with params " + msg.getStrategyParams() + " but ignored.");
        }
    }

    @Override
    public void onMobilityResponseMessage(MobilityResponse msg) {
        // Waiting on response for the current Candidate-Join plan
        if(this.currentPlan != null) {
            synchronized(currentPlan) {
                boolean isForCurrentPlan = msg.getHeader().getPlanId().equals(this.currentPlan.planId);
                boolean isFromTargetVehicle = msg.getHeader().getSenderId().equals(this.targetLeaderId);
                if(isForCurrentPlan && isFromTargetVehicle) {
                    if(msg.getIsAccepted()) {
                        // We change to follower state and start to actually follow that leader
                        // The platoon manager also need to change the platoon Id to the one that the target leader is using 
                        log.debug("The leader " + msg.getHeader().getSenderId() + " agreed on our join. Change to follower state.");
                        plugin.getPlatoonManager().changeFromLeaderToFollower(targetLeaderId, targetPlatoonId);
                        plugin.setState(new FollowerState(plugin, log, pluginServiceLocator));
                    } else {
                        // We change back to normal leader state and try to join other platoons
                        log.debug("The leader " + msg.getHeader().getSenderId() + " does not agree on our join. Change back to leader state.");
                        plugin.setState(new PlatoonLeaderState(plugin, log, pluginServiceLocator));
                    }
                } else {
                    log.debug("Ignore received response message because it is not for the current plan.");
                }
            }
        } else {
            log.debug("Ignore received response message because we are not in any negotiation process.");
        }
    }

    @Override
    public void run() {
        // This is a interrupted-safe loop.
        // This loop has four tasks:
        // 1. Check the state start time, if it exceeds a limit it will give up current plan and change back to leader state
        // 2. Abort current request if we wait for long enough time for response from leader and change back to leader state
        // 3. Check the current distance with the target platoon rear and send out CANDIDATE-JOIN request when we get close
        // 4. Publish operation status every 100 milliseconds if we still have followers
        try {
            while(!Thread.currentThread().isInterrupted()) {
                long tsStart = System.currentTimeMillis();
                // Task 1
                boolean isCurrentStateTimeout = (tsStart - this.stateStartTime) > plugin.getLongNegotiationTimeout();
                if(isCurrentStateTimeout) {
                    log.debug("The current candidate follower state is timeout. Change back to leader state.");
                    plugin.setState(new PlatoonLeaderState(plugin, log, pluginServiceLocator));
                }
                // Task 2
                if(this.currentPlan != null) {
                    synchronized (currentPlan) {
                        boolean isPlanTimeout = (tsStart - this.currentPlan.planStartTime) > plugin.getShortNegotiationTimeout();
                        if(isPlanTimeout) {
                            this.currentPlan = null;
                        }
                    }
                    log.debug("The current plan did not receive any response. Abort and change to leader state.");
                    plugin.setState(new PlatoonLeaderState(plugin, log, pluginServiceLocator));
                    
                }
                // Task 3
                if(pluginServiceLocator.getRouteService().getCurrentDowntrackDistance() >= speedUpEndDistance) {
                    MobilityRequest request = plugin.getMobilityRequestPublisher().newMessage();
                    String planId = UUID.randomUUID().toString();
                    request.getHeader().setPlanId(planId);
                    request.getHeader().setRecipientId(targetLeaderId);
                    request.getHeader().setSenderBsmId("FFFFFFFF");
                    request.getHeader().setSenderId(pluginServiceLocator.getMobilityRouter().getHostMobilityId());
                    request.getHeader().setTimestamp(System.currentTimeMillis());
                    // TODO Need to have a easy way to get current location in ECEF
                    request.getLocation().setEcefX(0);
                    request.getLocation().setEcefY(0);
                    request.getLocation().setEcefZ(0);
                    request.getLocation().setTimestamp(System.currentTimeMillis());
                    request.getPlanType().setType(PlanType.PLATOON_FOLLOWER_JOIN);
                    request.setStrategy(plugin.MOBILITY_STRATEGY);
                    request.setStrategyParams(String.format("DTD:%.2f",
                                              pluginServiceLocator.getRouteService().getCurrentDowntrackDistance()));
                    // TODO Maybe need to add some params (vehicle IDs) into strategy string
                    // TODO Maybe need to populate the urgency later
                    request.setUrgency((short) 50);
                    plugin.getMobilityRequestPublisher().publish(request);
                    log.debug("Published Mobility Candidate-Join request to the leader");
                    this.currentPlan = new PlatoonPlan(System.currentTimeMillis(), planId, targetLeaderId);
                }
                // Task 4
                if(plugin.getPlatoonManager().getPlatooningSize() != 0) {
                    MobilityOperation status = plugin.getMobilityOperationPublisher().newMessage();
                    composeMobilityOperationStatus(status);
                    plugin.getMobilityOperationPublisher().publish(status);
                }
                long tsEnd = System.currentTimeMillis();
                long sleepDuration = Math.max(plugin.getOperationUpdatesIntervalLength() - (tsEnd - tsStart), 0);
                Thread.sleep(sleepDuration);
            }
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
    
    
    @Override
    public String toString() {
        return "CandidateFollowerState";
    }
    
    // This method compose mobility operation STATUS message
    private void composeMobilityOperationStatus(MobilityOperation msg) {
        msg.getHeader().setPlanId(plugin.getPlatoonManager().getCurrentPlatoonID());
        // All platoon mobility operation message is just for broadcast
        msg.getHeader().setRecipientId("");
        // TODO Need to have a easy way to get bsmId from plugin
        msg.getHeader().setSenderBsmId("FFFFFFFF");
        String hostStaticId = pluginServiceLocator.getMobilityRouter().getHostMobilityId();
        msg.getHeader().setSenderId(hostStaticId);
        msg.getHeader().setTimestamp(System.currentTimeMillis());
        // TODO Strategy id in operation might be ready to removed
        msg.setStrategyId("");
        msg.setStrategy(plugin.MOBILITY_STRATEGY);
        // TODO Maneuver planner from plugin service locator may need to provide this data directly 
        SpeedAccel lastCmdSpeedObject = plugin.getCmdSpeedSub().getLastMessage();
        double cmdSpeed = lastCmdSpeedObject == null ? 0.0 : lastCmdSpeedObject.getSpeed();
        // For STATUS params, the string format is "STATUS|CMDSPEED:xx,DTD:xx,SPEED:xx"
        String statusParams = String.format("STATUS|CMDSPEED:%.2f,DTD:%.2f,SPEED:%.2f",
                                            cmdSpeed, pluginServiceLocator.getRouteService().getCurrentDowntrackDistance(),
                                            pluginServiceLocator.getManeuverPlanner().getManeuverInputs().getCurrentSpeed());
        msg.setStrategyParams(statusParams);
        log.debug("Composed a mobility operation message with params " + msg.getStrategyParams());
    }
}
