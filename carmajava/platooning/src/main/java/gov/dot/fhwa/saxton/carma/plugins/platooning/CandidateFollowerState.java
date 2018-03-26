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
import gov.dot.fhwa.saxton.carma.plugins.platooning.PlatoonPlan.PlanStatus;
import gov.dot.fhwa.saxton.carma.plugins.platooning.PlatoonPlan.PlatooningPlanType;

public class CandidateFollowerState implements IPlatooningState {

    protected static double LONGER_TRAJ_BUFFER_FACTOR = 1.5;
    
    protected PlatooningPlugin     plugin;
    protected ILogger              log;
    protected PluginServiceLocator pluginServiceLocator;
    private   double               maneuverEndDistance = Double.MAX_VALUE;
    private   double               speedUpTime;
    private   String               targetLeaderId;
    private   boolean              hasPlanned;
    private   PlatoonPlan          currentPlan;
    private   long                 stateStartTime;
    

    public CandidateFollowerState(PlatooningPlugin plugin, ILogger log, PluginServiceLocator pluginServiceLocator, double speedUpTime, String targetId) {
        this.plugin               = plugin;
        this.log                  = log;
        this.pluginServiceLocator = pluginServiceLocator;
        this.speedUpTime          = speedUpTime;
        this.hasPlanned           = false;
        this.targetLeaderId       = targetId;
        this.stateStartTime       = System.currentTimeMillis();
    }

    @Override
    public TrajectoryPlanningResponse planTrajectory(Trajectory traj, double expectedEntrySpeed) {
        RouteService rs = pluginServiceLocator.getRouteService();
        TrajectoryPlanningResponse tpr = new TrajectoryPlanningResponse();
        // If there is no platooning window, we set plugin to standby state
        if(!rs.isAlgorithmEnabledInRange(traj.getStartLocation(), traj.getEndLocation(), plugin.PLATOONING_FLAG)) {
            log.info("There is no platoon window in " + traj.toString() + " . Change back to standby state from " + this.toString());
            // TODO send out mobility operation message to inform LEAVE and delegate the leader job if necessary
            plugin.setState(new StandbyState(plugin, log, pluginServiceLocator));
            // If we have already planned a spped up trajectory but interrupted 
            if(hasPlanned) {
                // TODO maybe need to inform the front waiting leader to abort current plan
                log.debug("We have planned a speed up but the plan is interrupted due to no plan window avaliable");
            }
        } else {
            if(hasPlanned) {
                // TODO maybe need to inform the front waiting leader to abort current plan
                log.debug("We have planned a speed up but the plan is interrupted due to a re-plan event. Change back to PlatoonLeaderState");
                plugin.setState(new PlatoonLeaderState(plugin, log, pluginServiceLocator));
            } else {
                // Plan a speed up maneuver to speed limit and
                // a steady speed maneuver at the speed limit for a period of time indicated by speedUpTime
                SimpleManeuverFactory maneuverFactory = new SimpleManeuverFactory(plugin);
                double currentSpeed = pluginServiceLocator.getManeuverPlanner().getManeuverInputs().getCurrentSpeed();
                double speedLimit = rs.getSpeedLimitAtLocation(rs.getCurrentDowntrackDistance()).getLimit();
                LongitudinalManeuver speedUp = maneuverFactory.createManeuver(currentSpeed, speedLimit);
                speedUp.setSpeeds(currentSpeed, speedLimit);
                speedUp.setMaxAccel(plugin.getMaxAccel());
                pluginServiceLocator.getManeuverPlanner().planManeuver(speedUp, traj.getStartLocation());
                double speedUpEndDtd = speedUp.getEndDistance();
                log.debug("Planned a speedUp maneuver ends at " + speedUpEndDtd);
                double steadySpeedManeuverEnd = speedUpEndDtd + (speedLimit * this.speedUpTime);
                maneuverEndDistance = steadySpeedManeuverEnd;
                log.debug("If we plan a steady speed maneuver to catch up the gap, we need trajectory to end at least " + steadySpeedManeuverEnd);
                if(steadySpeedManeuverEnd > traj.getEndLocation()) {
                    log.debug("steadySpeedManeuverEnd exceeds the end of current trajectory, requesting a longer one.");
                    tpr.requestLongerTrajectory(LONGER_TRAJ_BUFFER_FACTOR * (steadySpeedManeuverEnd - traj.getStartLocation()));
                } else {
                    LongitudinalManeuver steadySpeed = maneuverFactory.createManeuver(speedLimit, speedLimit);
                    steadySpeed.setSpeeds(speedLimit, speedLimit);
                    steadySpeed.setMaxAccel(plugin.getMaxAccel());
                    pluginServiceLocator.getManeuverPlanner().planManeuver(steadySpeed, speedUpEndDtd);
                    ((SteadySpeed) steadySpeed).overrideEndDistance(steadySpeedManeuverEnd);
                    log.debug("Planned steady speed maneuver which ends at " + steadySpeed.getEndDistance());
                    boolean isSpeedUpInserted = traj.addManeuver(speedUp);
                    boolean isSteadySpeedInserted = traj.addManeuver(steadySpeed);
                    if(isSpeedUpInserted && isSteadySpeedInserted) {
                        log.debug("Inserted speed up maneuver and also steady speed maneuver to close the gap.");
                        this.hasPlanned = true;
                    } else {
                        log.debug("Insertion of speed up maneuver and steady speed maneuver fails:");
                        log.debug("isSpeedUpInserted=" + isSpeedUpInserted + ", isSteadySpeedInserted=" + isSteadySpeedInserted);
                        tpr.requestHigherPriority();
                    }
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
        // we only handle STATUS opertion message from our platoonn
        String strategyParams = msg.getStrategyParams();
        boolean isPlatoonStatusMsg = strategyParams.startsWith("STATUS");
        boolean isFromOurPlatoon = msg.getStrategyId().equals(plugin.getPlatoonManager().getCurrentPlatoonID());
        if(isPlatoonStatusMsg && isFromOurPlatoon) {
            // TODO we need to decide as the leader, what information we should maintain for this platoon
            log.debug("Received platoon status message from our platoon from " + msg.getHeader().getSenderId());
        }
    }

    @Override
    public void onMobilityResponseMessage(MobilityResponse msg) {
        // Waiting on response for the current Candidate-join plan
        if(this.currentPlan != null && this.currentPlan.status == PlanStatus.WAITING_ON_RESPONSE) {
            boolean isForCurrentPlan = msg.getHeader().getPlanId().equals(this.currentPlan.planId);
            if(isForCurrentPlan) {
                if(msg.getIsAccepted()) {
                    // we change to follower state and start to actually follow that leader
                    log.debug("The leader " + msg.getHeader().getSenderId() + " agreed on our join. Change to follower state.");
                    plugin.setState(new FollowerState(plugin, log, pluginServiceLocator));
                    // TODO maybe to reset leader on platoon manager
                } else {
                    // we change back to leader state and try to join other platoon
                    log.debug("The leader " + msg.getHeader().getSenderId() + " does not agree on our join. Change back to leader state.");
                    plugin.setState(new PlatoonLeaderState(plugin, log, pluginServiceLocator));
                }
            } else {
                log.debug("Ignore received response message because it is not for the current plan.");
            }
        } else {
            log.debug("Ignore received response message because we are not in any negotiation process.");
        }
    }

    @Override
    public void run() {
        // This is a interrupted-safe loop
        // This loop does four things:
        // 1. Check the state start time, if execeeds a limit it will give up joining and change back to leader state
        // 2. Abort current plan if we wait for long enough time and change back to leader state
        // 3. Check the current distance with the target platoon rear and send out CANDIDATE-JOIN request
        // 4. Publish operation status every 100 milliseconds if necessary
        try {
            while(!Thread.currentThread().isInterrupted()) {
                long tsStart = System.currentTimeMillis();
                // Job No.1
                boolean isCurrentStateTimeout = (tsStart - this.stateStartTime) > plugin.getLongNegotiationTimeout();
                if(isCurrentStateTimeout) {
                    log.debug("The current candidate follower state is timeout. Change to leader state.");
                    plugin.setState(new PlatoonLeaderState(plugin, log, pluginServiceLocator));
                }
                // Job No.2
                if(this.currentPlan != null) {
                    boolean isPlanTimeout = (tsStart - this.currentPlan.planStartTime) > plugin.getShortNegotiationTimeout();
                    this.currentPlan = null;
                    log.debug("The current plan did not receive any response. Abort and change to leader state.");
                    plugin.setState(new PlatoonLeaderState(plugin, log, pluginServiceLocator));
                }
                // Job No.3
                if(pluginServiceLocator.getRouteService().getCurrentDowntrackDistance() > maneuverEndDistance) {
                    MobilityRequest request = plugin.getMobilityRequestPublisher().newMessage();
                    String planId = UUID.randomUUID().toString();
                    request.getHeader().setPlanId(planId);
                    request.getHeader().setRecipientId(targetLeaderId);
                    request.getHeader().setSenderBsmId("FFFFFFFF");
                    request.getHeader().setSenderId(pluginServiceLocator.getMobilityRouter().getHostMobilityId());
                    request.getHeader().setTimestamp(System.currentTimeMillis());
                    // TODO need to have a easy way to get current location in ECEF
                    request.getPlanType().setType(PlanType.PLATOON_FOLLOWER_JOIN);
                    request.setStrategy(plugin.MOBILITY_STRATEGY);
                    // TODO maybe need to add some params (vehicle ID) into strategy string
                    // TODO need to populate the urgency later
                    request.setUrgency((short) 50);
                    plugin.getMobilityRequestPublisher().publish(request);
                    log.debug("Published MObility Candidate-Join request to the leader");
                    this.currentPlan = new PlatoonPlan(System.currentTimeMillis(), planId, PlanStatus.WAITING_ON_RESPONSE, PlatooningPlanType.CANDIDATE_FOLLOWER_JOIN);
                }
                // Job No.4
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
        msg.getHeader().setPlanId("");
        // This message is for broadcast
        msg.getHeader().setRecipientId("");
        // TODO need to have a easy way to get bsmId in plugin
        msg.getHeader().setSenderBsmId("FFFFFFFF");
        String hostStaticId = pluginServiceLocator.getMobilityRouter().getHostMobilityId();
        msg.getHeader().setSenderId(hostStaticId);
        msg.getHeader().setTimestamp(System.currentTimeMillis());
        msg.setStrategyId(plugin.getPlatoonManager().getCurrentPlatoonID());
        msg.setStrategy(plugin.MOBILITY_STRATEGY);
        // For STATUS params, the string format is "STATUS|CMDSPEED:5.0,DOWNTRACK:100.0,SPEED:5.0"
        SpeedAccel lastCmdSpeed = plugin.getCmdSpeedSub().getLastMessage();
        double cmdSpeed = lastCmdSpeed == null ? 0.0 : lastCmdSpeed.getSpeed();
        double downtrackDistance = pluginServiceLocator.getRouteService().getCurrentDowntrackDistance();
        double currentSpeed = pluginServiceLocator.getManeuverPlanner().getManeuverInputs().getCurrentSpeed();
        String params = String.format("STATUS|CMDSPEED:%.2f,DOWNTRACK:%.2f,SPEED:%.2f", cmdSpeed, downtrackDistance, currentSpeed);
        msg.setStrategyParams(params);
    }
}
