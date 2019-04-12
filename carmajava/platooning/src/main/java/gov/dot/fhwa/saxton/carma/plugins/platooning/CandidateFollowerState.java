package gov.dot.fhwa.saxton.carma.plugins.platooning;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;
import java.util.SortedSet;
import java.util.UUID;

import cav_msgs.MobilityOperation;
import cav_msgs.MobilityRequest;
import cav_msgs.MobilityResponse;
import cav_msgs.PlanType;
import gov.dot.fhwa.saxton.carma.guidance.ManeuverPlanner;
import gov.dot.fhwa.saxton.carma.guidance.arbitrator.TrajectoryPlanningResponse;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.LongitudinalManeuver;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.SimpleManeuverFactory;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.SteadySpeed;
import gov.dot.fhwa.saxton.carma.guidance.mobilityrouter.MobilityRequestResponse;
import gov.dot.fhwa.saxton.carma.guidance.plugins.PluginServiceLocator;
import gov.dot.fhwa.saxton.carma.guidance.trajectory.Trajectory;
import gov.dot.fhwa.saxton.carma.guidance.util.ILogger;
import gov.dot.fhwa.saxton.carma.guidance.util.RouteService;
import gov.dot.fhwa.saxton.carma.guidance.util.SpeedLimit;
import gov.dot.fhwa.saxton.carma.guidance.util.trajectoryconverter.RoutePointStamped;

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

    protected static final double SPEED_EPSILON = 0.1;
    protected static final double LONGER_TRAJ_BUFFER_FACTOR = 1.5;
    
    protected PlatooningPlugin     plugin;
    protected ILogger              log;
    protected PluginServiceLocator pluginServiceLocator;
    protected String               targetLeaderId;
    protected String               targetPlatoonId;
    private   double               trajectoryEndLocation;
    private   PlatoonPlan          currentPlan;
    private   long                 stateStartTime;

    public CandidateFollowerState(PlatooningPlugin plugin, ILogger log, PluginServiceLocator pluginServiceLocator, String targetId, String newPlatoonId, double trajectoryEnd) {
        this.plugin                = plugin;
        this.log                   = log;
        this.pluginServiceLocator  = pluginServiceLocator;
        this.targetLeaderId        = targetId;
        this.targetPlatoonId       = newPlatoonId;
        this.trajectoryEndLocation = trajectoryEnd;
        this.stateStartTime        = System.currentTimeMillis();
        this.plugin.handleMobilityPath.set(false);
    }

    @Override
    public TrajectoryPlanningResponse planTrajectory(Trajectory traj, double expectedEntrySpeed) {
        this.trajectoryEndLocation = traj.getEndLocation();
        RouteService rs = pluginServiceLocator.getRouteService();
        TrajectoryPlanningResponse tpr = new TrajectoryPlanningResponse();
        // If there is no platooning window, we set plugin to Standby state
        if(!rs.isAlgorithmEnabledInRange(traj.getStartLocation(), traj.getEndLocation(), PlatooningPlugin.PLATOONING_FLAG)) {
            log.debug("There is no platoon window in " + traj.toString() + " . Change back to standby state");
            // TODO Send out mobility request message to inform its LEAVE and delegate the leader job properly
            plugin.setState(new StandbyState(plugin, log, pluginServiceLocator));
        } else {
            // Plan a speed up maneuver to speed limit and a steady speed maneuver at the speed limit
            // TODO the following logic is from cruising plugin, I feel uncomfortable to re-use it here
            // TODO the following logic should be moved to route service
            SortedSet<SpeedLimit> trajLimits = rs.getSpeedLimitsInRange(traj.getStartLocation(), traj.getEndLocation());
            trajLimits.add(rs.getSpeedLimitAtLocation(traj.getEndLocation()));
            // Merge segments with same speed limits
            List<SpeedLimit> mergedLimits = new LinkedList<SpeedLimit>();
            SpeedLimit limit_buffer = null;
            for (SpeedLimit limit : trajLimits) {
                // Create new object to prevent speed limit modification
                SpeedLimit followedLimit = new SpeedLimit(limit.getLocation(), limit.getLimit());
                // Merge segments with same speed
                if (limit_buffer == null) {
                    limit_buffer = followedLimit;
                } else {
                    if (Math.abs(limit_buffer.getLimit() - followedLimit.getLimit()) < SPEED_EPSILON) {
                        limit_buffer.setLocation(followedLimit.getLocation());
                    } else {
                        mergedLimits.add(limit_buffer);
                        limit_buffer = followedLimit;
                    }
                }
            }
            limit_buffer.setLocation(traj.getEndLocation());
            mergedLimits.add(limit_buffer);
            
            // Plan trajectory to follow all speed limits in this trajectory segment
            double newManeuverStartSpeed = expectedEntrySpeed;
            if(mergedLimits.size() > 0 && mergedLimits.get(0).getLimit() < newManeuverStartSpeed) {
                newManeuverStartSpeed = mergedLimits.get(0).getLimit();
                log.warn("Current speed is exceeding the local speed limit!!! Cap the trajectory start speed to the speed limit.");
            }
            double newManeuverStartLocation = traj.getStartLocation();
            for (SpeedLimit limit : mergedLimits) {
                newManeuverStartSpeed = planManeuvers(traj, newManeuverStartLocation, limit.getLocation(), newManeuverStartSpeed, limit.getLimit());
                newManeuverStartLocation = limit.getLocation();
            }
        }
        return tpr;
    }

    private double planManeuvers(Trajectory t, double startDist, double endDist, double startSpeed, double endSpeed) {
        ManeuverPlanner planner = pluginServiceLocator.getManeuverPlanner();
        SimpleManeuverFactory maneuverFactory = new SimpleManeuverFactory(plugin);
        log.info(String.format("Platooning is trying to plan maneuver {start=%.2f,end=%.2f,startSpeed=%.2f,endSpeed=%.2f} to close the gap",
                 startDist, endDist, startSpeed, endSpeed));
        double maneuverEnd = startDist;
        double adjustedEndSpeed = startSpeed;
        LongitudinalManeuver maneuver = maneuverFactory.createManeuver(startSpeed, endSpeed);
        maneuver.setSpeeds(startSpeed, endSpeed);
        maneuver.setMaxAccel(plugin.maxAccel);
        if(planner.canPlan(maneuver, startDist, endDist)) {
            planner.planManeuver(maneuver, startDist);
            if(maneuver instanceof SteadySpeed) {
                ((SteadySpeed) maneuver).overrideEndDistance(endDist);
            }
            if(maneuver.getEndDistance() > endDist) {
                adjustedEndSpeed = planner.planManeuver(maneuver, startDist, endDist);
            } else {
                adjustedEndSpeed = maneuver.getTargetSpeed();
            }
            maneuverEnd = maneuver.getEndDistance();
            t.addManeuver(maneuver);
            log.info(String.format("Platooning has planned maneuver from [%.2f, %.2f) m/s over [%.2f, %.2f) m to close the gap", startSpeed, adjustedEndSpeed, startDist, maneuverEnd));
        }
        
        // Insert a steady speed maneuver to fill whatever's left
        if(maneuverEnd < endDist) {
          LongitudinalManeuver steady = maneuverFactory.createManeuver(adjustedEndSpeed, adjustedEndSpeed);
            steady.setSpeeds(adjustedEndSpeed, adjustedEndSpeed);
            steady.setMaxAccel(plugin.maxAccel);
            planner.planManeuver(steady, maneuverEnd);
            ((SteadySpeed) steady).overrideEndDistance(endDist);
            t.addManeuver(steady);
            log.info(String.format("Platooning planned a STEADY-SPEED maneuver at %.2f m/s over [%.2f, %.2f) m to close the gap", adjustedEndSpeed, maneuverEnd, endDist));
        }
        return adjustedEndSpeed;
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
        boolean isPlatoonStatusMsg = strategyParams.startsWith(PlatooningPlugin.OPERATION_STATUS_TYPE);
        if(isPlatoonStatusMsg) {
            String vehicleID = msg.getHeader().getSenderId();
            String platoonId = msg.getHeader().getPlanId();
            String statusParams = strategyParams.substring(PlatooningPlugin.OPERATION_STATUS_TYPE.length() + 1);
            plugin.platoonManager.memberUpdates(vehicleID, platoonId, msg.getHeader().getSenderBsmId(), statusParams);
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
                if(this.currentPlan != null) {
                    boolean isForCurrentPlan = msg.getHeader().getPlanId().equals(this.currentPlan.planId);
                    boolean isFromTargetVehicle = msg.getHeader().getSenderId().equals(this.targetLeaderId);
                    if(isForCurrentPlan && isFromTargetVehicle) {
                        if(msg.getIsAccepted()) {
                            // We change to follower state and start to actually follow that leader
                            // The platoon manager also need to change the platoon Id to the one that the target leader is using 
                            log.debug("The leader " + msg.getHeader().getSenderId() + " agreed on our join. Change to follower state.");
                            plugin.platoonManager.changeFromLeaderToFollower(targetPlatoonId);
                            plugin.setState(new FollowerState(plugin, log, pluginServiceLocator));
                            pluginServiceLocator.getArbitratorService().requestNewPlan(this.trajectoryEndLocation);
                        } else {
                            // We change back to normal leader state and try to join other platoons
                            log.debug("The leader " + msg.getHeader().getSenderId() + " does not agree on our join. Change back to leader state.");
                            plugin.setState(new LeaderState(plugin, log, pluginServiceLocator));
                        }
                    } else {
                        log.debug("Ignore received response message because it is not for the current plan.");
                    }    
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
                boolean isCurrentStateTimeout = (tsStart - this.stateStartTime) > plugin.waitingStateTimeout * 1000;
                if(isCurrentStateTimeout) {
                    log.debug("The current candidate follower state is timeout. Change back to leader state.");
                    plugin.setState(new LeaderState(plugin, log, pluginServiceLocator));
                }
                // Task 2
                if(this.currentPlan != null) {
                    synchronized (currentPlan) {
                        if(this.currentPlan != null) {
                            boolean isPlanTimeout = (tsStart - this.currentPlan.planStartTime) > PlatooningPlugin.NEGOTIATION_TIMEOUT;
                            if(isPlanTimeout) {
                                this.currentPlan = null;
                                log.debug("The current plan did not receive any response. Abort and change to leader state.");
                                plugin.setState(new LeaderState(plugin, log, pluginServiceLocator));
                            }    
                        }
                    }
                }
                // Task 3
                double desiredJoinGap = plugin.desiredJoinTimeGap * plugin.getManeuverInputs().getCurrentSpeed();
                double maxJoinGap = Math.max(plugin.desiredJoinGap, desiredJoinGap);
                double currentGap = plugin.getManeuverInputs().getDistanceToFrontVehicle();
                log.debug("Based on desired join time gap, the desired join distance gap is " + desiredJoinGap + " ms");
                log.debug("Since we have max allowed gap as " + plugin.desiredJoinGap + " m then max join gap became " + maxJoinGap + " m");
                log.debug("The current gap from radar is " + currentGap + " m");
                if(currentGap <= maxJoinGap && this.currentPlan == null) {
                    MobilityRequest request = plugin.mobilityRequestPublisher.newMessage();
                    String planId = UUID.randomUUID().toString();
                    long currentTime = System.currentTimeMillis();
                    request.getHeader().setPlanId(planId);
                    request.getHeader().setRecipientId(targetLeaderId);
                    request.getHeader().setSenderBsmId(pluginServiceLocator.getTrackingService().getCurrentBSMId());
                    request.getHeader().setSenderId(pluginServiceLocator.getMobilityRouter().getHostMobilityId());
                    request.getHeader().setTimestamp(currentTime);
                    RoutePointStamped currentLocation = new RoutePointStamped(plugin.getManeuverInputs().getDistanceFromRouteStart(),
                    plugin.getManeuverInputs().getCrosstrackDistance(), currentTime / 1000.0);
                    cav_msgs.Trajectory currentLocationMsg = pluginServiceLocator.getTrajectoryConverter().pathToMessage(Arrays.asList(currentLocation));
                    request.setLocation(currentLocationMsg.getLocation());
                    request.getPlanType().setType(PlanType.PLATOON_FOLLOWER_JOIN);
                    request.setStrategy(PlatooningPlugin.MOBILITY_STRATEGY);
                    request.setStrategyParams("");
                    // TODO Maybe need to add some params (vehicle IDs) into strategy string
                    // TODO Maybe need to populate the urgency later
                    request.setUrgency((short) 50);
                    plugin.mobilityRequestPublisher.publish(request);
                    log.debug("Published Mobility Candidate-Join request to the leader");
                    this.currentPlan = new PlatoonPlan(System.currentTimeMillis(), planId, targetLeaderId);
                }
                // Task 4
                if(plugin.platoonManager.getTotalPlatooningSize() > 1) {
                    MobilityOperation status = plugin.mobilityOperationPublisher.newMessage();
                    composeMobilityOperationStatus(status);
                    plugin.mobilityOperationPublisher.publish(status);
                }
                long tsEnd = System.currentTimeMillis();
                long sleepDuration = Math.max(plugin.statusMessageInterval - (tsEnd - tsStart), 0);
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
        msg.getHeader().setPlanId(plugin.platoonManager.currentPlatoonID);
        // All platoon mobility operation message is just for broadcast
        msg.getHeader().setRecipientId("");
        // TODO Need to have a easy way to get bsmId from plugin
        msg.getHeader().setSenderBsmId(pluginServiceLocator.getTrackingService().getCurrentBSMId());
        String hostStaticId = pluginServiceLocator.getMobilityRouter().getHostMobilityId();
        msg.getHeader().setSenderId(hostStaticId);
        msg.getHeader().setTimestamp(System.currentTimeMillis());
        msg.setStrategy(PlatooningPlugin.MOBILITY_STRATEGY);
        double cmdSpeed = plugin.getLastSpeedCmd();
        // For STATUS params, the string format is "STATUS|CMDSPEED:xx,DTD:xx,SPEED:xx"
        String statusParams = String.format(PlatooningPlugin.OPERATION_STATUS_PARAMS,
                                            cmdSpeed, pluginServiceLocator.getRouteService().getCurrentDowntrackDistance(),
                                            pluginServiceLocator.getManeuverPlanner().getManeuverInputs().getCurrentSpeed());
        msg.setStrategyParams(statusParams);
        log.debug("Composed a mobility operation message with params " + msg.getStrategyParams());
    }
}
