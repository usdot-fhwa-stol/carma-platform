/*
 * Copyright (C) 2018 LEIDOS.
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

package gov.dot.fhwa.saxton.carma.plugins.platooning;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.UUID;

import gov.dot.fhwa.saxton.carma.guidance.maneuvers.IManeuverInputs;
import gov.dot.fhwa.saxton.carma.guidance.plugins.PluginServiceLocator;
import gov.dot.fhwa.saxton.carma.guidance.util.ILogger;

/**
 * This class manages the info of members in the current platoon and leader selection process.
 * In leader state, the manager will maintain a full list of followers' information.
 * In follower state, the manager will keep a list of platoon members' information who is in front of the host vehicle. 
 */
public class PlatoonManager implements Runnable {
    
    protected PlatooningPlugin     plugin;
    protected ILogger              log;
    protected PluginServiceLocator psl;
    private   long                 memberInfoTimeout;
    private   boolean              isFollower;
    
    
    // This platoon list will have different usages under leader/follower state
    protected List<PlatoonMember>  platoon             = Collections.synchronizedList(new ArrayList<>());
    protected String               currentPlatoonID    = UUID.randomUUID().toString();
    protected String               leaderID            = "";
    
    protected volatile int         platoonCurrentSize = 1;
    
    // The following variables are used for APF leader selection algorithm
    protected String               previousFunctionalLeaderID    = "";
    protected int                  previousFunctionalLeaderIndex = -1;

    public PlatoonManager(PlatooningPlugin plugin, ILogger log, PluginServiceLocator psl) {
        this.plugin            = plugin;
        this.log               = log;
        this.psl               = psl;
        this.memberInfoTimeout = (long) (plugin.getOperationUpdatesIntervalLength() * plugin.getOperationUpdatesTimeoutFactor());
        this.isFollower        = false;
    }
    
    /**
     * Given any valid platooning mobility STATUS operation params and sender staticId,
     * in leader state this method will add/updates the information of platoon member if it is using
     * the same strategy ID, in follower state this method will updates the vehicle information who
     * is in front of the subject vehicle and also updates the platoon ID information if necessary
     * @param senderId Sender ID for the current info
     * @param params Strategy params from STATUS message in the format of "CMDSPEED:xx,DOWNTRACK:xx,SPEED:xx"
     */
    protected synchronized void memberUpdates(String senderId, String platoonId, String params) {
        String[] inputsParams = params.split(",");
        // TODO we should get downtrack distance for other vehicle from either roadway environment or
        // from strategy params in the ECEF frame, but not directly from this string
        double cmdSpeed   = Double.parseDouble(inputsParams[0].split(":")[1]);
        double dtDistance = Double.parseDouble(inputsParams[1].split(":")[1]);
        double curSpeed   = Double.parseDouble(inputsParams[2].split(":")[1]);
        // If we are currently in a follower state:
        // 1. We will update platoon ID based on leader's STATUS
        // 2. We will update platoon members info based on platoon ID if it is in front of us 
        if(isFollower) {
            boolean isFromLeader = leaderID.equals(senderId);
            boolean needPlatoonIdChange = isFromLeader && !this.currentPlatoonID.equals(platoonId);
            boolean isInFrontOfUs = dtDistance >= psl.getRouteService().getCurrentDowntrackDistance();
            // Task 1
            if(needPlatoonIdChange) {
                log.debug("It seems that the current leader is joining another platoon.");
                log.debug("So the platoon ID is changed from " + this.currentPlatoonID + " to " + platoonId);
                this.setCurrentPlatoonID(platoonId);
                updatesOrAddMemberInfo(senderId, cmdSpeed, dtDistance, curSpeed);
            } else if(this.currentPlatoonID.equals(platoonId) && isInFrontOfUs) {
                log.debug("This STATUS messages is from our platoon in front of us. Updating the info...");
                updatesOrAddMemberInfo(senderId, cmdSpeed, dtDistance, curSpeed);
                if(!platoon.isEmpty() && !this.leaderID.equals(platoon.get(0).staticId)) {
                    this.leaderID = platoon.get(0).staticId;
                    log.debug("Now the leader change to " + this.leaderID);
                }
            } else {
                log.debug("This STATUS message is not from our platoon. We ignore this message with id: " + senderId);
            }
        } else {
            // If we are currently in any leader state, we only updates platoon member based on platoon ID
            if(currentPlatoonID.equals(platoonId)) {
                log.debug("This STATUS messages is from our platoon. Updating the info...");
                updatesOrAddMemberInfo(senderId, cmdSpeed, dtDistance, curSpeed);
            }
        }
    }
    
    private void updatesOrAddMemberInfo(String senderId, double cmdSpeed, double dtDistance, double curSpeed) {
        boolean isExisted = false;
        // update/add this info into the list
        for(PlatoonMember pm : platoon) {
            if(pm.staticId.equals(senderId)) {
                pm.commandSpeed = cmdSpeed;
                pm.vehiclePosition = dtDistance;
                pm.vehicleSpeed = curSpeed;
                pm.timestamp = System.currentTimeMillis();
                log.debug("Receive and update platooning info on vehicel " + pm.staticId);
                log.debug("    Speed = "                             + pm.vehicleSpeed);
                log.debug("    Location = "                          + pm.vehiclePosition);
                log.debug("    CommandSpeed = "                      + pm.commandSpeed);
                isExisted = true;
                break;
            }
        }
        if(!isExisted) {
            PlatoonMember newMember = new PlatoonMember(senderId, cmdSpeed, curSpeed, dtDistance, System.currentTimeMillis());
            platoon.add(newMember);
            Collections.sort(platoon, (a, b) -> (Double.compare(b.vehiclePosition, a.vehiclePosition)));
            log.debug("Add a new vehicle into our platoon list " + newMember.staticId);
        }
    }
    
    private void setCurrentPlatoonID(String newPlatoonID) {
        log.info("Platoon ID is changed from " + this.currentPlatoonID + " to " + newPlatoonID);
        this.currentPlatoonID = newPlatoonID;
    }
    
    @Override
    public void run() {
        try {
            while(!Thread.currentThread().isInterrupted()) {
                long loopStart = System.currentTimeMillis();
                removeExpiredMember();
                long loopEnd = System.currentTimeMillis();
                long sleepDuration = Math.max(this.memberInfoTimeout - (loopEnd - loopStart), 0);
                Thread.sleep(sleepDuration);
            }
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
    
    // This method removes any expired/invalid entries from platoon list
    protected synchronized void removeExpiredMember() {
        List<PlatoonMember> removeCandidates = new ArrayList<>();
        for(PlatoonMember pm : platoon) {
            boolean isTimeout = System.currentTimeMillis() - pm.timestamp > memberInfoTimeout;
            if(isTimeout) {
                removeCandidates.add(pm);
                log.debug("Found invalid vehicel entry " + pm.staticId + " in platoon list which will be removed");
                log.debug("Because isTimeout = " + isTimeout);
            }
        }
        if(removeCandidates.size() != 0) {
            for(PlatoonMember candidate: removeCandidates) {
                platoon.remove(candidate);
            }
        }
    }
    
    protected synchronized int getPlatooningSize() {
        return platoon.size();
    }
    
    protected synchronized double getPlatoonRearDowntrackDistance() {
        if(this.getPlatooningSize() == 0) {
            return psl.getRouteService().getCurrentDowntrackDistance();
        }
        return this.platoon.get(this.getPlatooningSize() - 1).vehiclePosition;
    }
    
    protected synchronized void changeFromLeaderToFollower(String newLeaderId, String newPlatoonId) {
        this.isFollower = true;
        this.leaderID = newLeaderId;
        this.currentPlatoonID = newPlatoonId;
        this.platoon = Collections.synchronizedList(new ArrayList<>());
        log.debug("The platoon manager is changed from leader state to follower state.");
    }
    
    protected synchronized void changeFromFollowerToLeader() {
        this.isFollower = false;
        this.platoon = Collections.synchronizedList(new ArrayList<>());
        log.debug("The platoon manager is changed from follower state to leader state.");
    }

    protected synchronized String getCurrentPlatoonID() {
        return currentPlatoonID;
    }
    
    /**
     * This method contains will use the indicated algorithm to determine
     * which vehicle in the platoon will function as the leader.
     * CommandGenerator will use the output of this function as the baseline cmd_speed 
     */
    protected synchronized PlatoonMember getLeader() {
        PlatoonMember leader = null;
        if(isFollower && platoon.size() != 0) {
            // return the first vehicle in the platoon as default if no valid algorithm applied
            leader = platoon.get(0);
            if(plugin.getAlgorithmType() == 1) {
                int newLeaderIndex = allPredecessorFollowing();
                if(newLeaderIndex < platoon.size() && newLeaderIndex >= 0) {
                    leader = platoon.get(newLeaderIndex);
                    log.debug("APF output: " + leader.staticId);
                    previousFunctionalLeaderIndex = newLeaderIndex;
                    previousFunctionalLeaderID = leader.staticId;
                } else {
                    // it might happened when the subject vehicle gets far away from the preceding vehicle
                    // In that case, it is OK to follow the first vehicle as leader
                    // TODO if we encounter any troubles, we will solve it by adding LEAVE request and also leave the platoon
                    log.warn("Cannot find the correct leader information based on the output of APF algorithm: " + newLeaderIndex);
                }
            }
            return leader;
        }
        return null;
    }
    
    /**
     * This is the implementation of all predecessor following (APF) algorithm for leader
     * selection in a platoon. This function will recognize who is acting as the current leader
     * of the subject vehicle. The current leader of the subject vehicle will be any ONE of
     * the vehicles in front of it. Having a vehicle further downstream function as the leader
     * is more efficient and more stable; however, having a vehicle closer to the subject vehicle
     * function as the leader is safer. For this reason, the subject vehicle will monitor
     * all time headways between every single set of consecutive vehicles starting from itself
     * to the leader. If the time headways are within some safe thresholds then vehicles further
     * downstream may function as the leader. Otherwise, for the sake of safety, vehicles closer
     * to the subject vehicle, potentially even the predecessor, will function as the leader.
     * @return the index of the leader in the platoon list
     */
    private int allPredecessorFollowing() {
        IManeuverInputs inputs = this.plugin.getManeuverInputs();
        ///***** Case Zero *****///
        // If we are the second vehicle in this platoon, we will always follow the leader vehicle
        if(platoon.size() == 1) {
            log.debug("As the second vehicle in the platoon, it will always follow the leader. Case Zero");
            return 0;
        }
        ///***** Case One *****///
        // If we do not have a leader in the previous time step, we follow the first vehicle as default 
        if(previousFunctionalLeaderID.equals("")) {
            log.debug("APF algorithm did not found a leader in previous time step. Case one");
            return 0;
        }
        // Generate an array of downtrack distance for every vehicles in this platoon including the host vehicle
        // The size of distance array is platoon.size() + 1, because the platoon list did not contain the host vehicle
        double[] downtrackDistance = new double[platoon.size() + 1];
        for(int i = 0; i < platoon.size(); i++) {
            downtrackDistance[i] = platoon.get(i).vehiclePosition; 
        }
        downtrackDistance[downtrackDistance.length - 1] = inputs.getDistanceFromRouteStart();
        
        // Generate an array of speed for every vehicles in this platoon including the host vehicle
        // The size of speed array is platoon.size() + 1, because the platoon list did not contain the host vehicle
        double[] speed = new double[platoon.size() + 1];
        for(int i = 0; i < platoon.size(); i++) {
            speed[i] = platoon.get(i).vehicleSpeed;
        }
        speed[speed.length - 1] = inputs.getCurrentSpeed();
        ///***** Case Two *****///
        // If the distance headway between the subject vehicle and its predecessor is an issue
        // according to the "min_gap" and "max_gap" thresholds, then it should follow its predecessor
        if(insufficientGapWithPredecessor(inputs.getDistanceToFrontVehicle())) {
            log.debug("APF algorithm decides there is an issue with the gap with preceding vehicle. Case Two");
            return platoon.size() - 1;
        } else {
            // implementation of the main part of APF algorithm
            // calculate the time headway between every consecutive pair of vehicles
            double[] timeHeadways = calculateTimeHeadway(downtrackDistance, speed);
            log.debug("APF calculate time headways: " + Arrays.toString(timeHeadways));
            log.debug("APF found the previous leader is " + previousFunctionalLeaderID);
            // if the previous leader is the first vehicle in the platoon
            if(previousFunctionalLeaderIndex == 0) {
                ///***** Case Three *****///
                // If there is a violation, the return value is the desired leader index
                log.debug("APF use violations on lower boundary or maximum spacing to choose leader. Case Three.");
                return determineLeaderBasedOnViolation(timeHeadways);
            } else {
                // if the previous leader is not the first vehicle
                // get the time headway between every consecutive pair of vehicles from indexOfPreviousLeader
                double[] partialTimeHeadways = getTimeHeadwayFromIndex(timeHeadways, previousFunctionalLeaderIndex);
                int closestLowerBoundaryViolation, closestMaximumSpacingViolation;
                closestLowerBoundaryViolation = findLowerBoundaryViolationClosestToTheHostVehicle(partialTimeHeadways);
                closestMaximumSpacingViolation = findMaximumSpacingViolationClosestToTheHostVehicle(partialTimeHeadways);
                // if there are no violations anywhere between the subject vehicle and the current leader,
                // then depending on the time headways of the ENTIRE platoon, the subject vehicle may switch
                // leader further downstream. This is because the subject vehicle has determined that there are
                // no time headways between itself and the current leader which would cause the platoon to be unsafe.
                // if there are violations somewhere betweent the subject vehicle and the current leader,
                // then rather than assigning leadership further DOWNSTREAM, we must go further UPSTREAM in the following lines
                if(closestLowerBoundaryViolation == -1 && closestMaximumSpacingViolation == -1) {
                    // In order for the subject vehicle to assign leadership further downstream,
                    // two criteria must be satisfied: first the leading vehicle and its immediate follower must
                    // have a time headway greater than "upper_boundary." The purpose of this criteria is to
                    // introduce a hysteresis in order to eliminate the possibility of a vehicle continually switching back 
                    // and forth between two leaders because one of the time headways is hovering right around
                    // the "lower_boundary" threshold; second the leading vehicle and its predecessor must have
                    // a time headway less than "min_spacing" second. Just as with "upper_boundary", "min_spacing" exists to
                    // introduce a hysteresis where leaders are continually being switched.
                    boolean condition1 = timeHeadways[previousFunctionalLeaderIndex] > plugin.getUpperBoundary();
                    boolean condition2 = timeHeadways[previousFunctionalLeaderIndex - 1] < plugin.getMinSpacing();
                    ///***** Case Four *****///
                    //we may switch leader further downstream
                    if(condition1 && condition2) {
                        log.debug("APF found two conditions for assigning leadership further downstream are satisfied. Case Four");
                        return determineLeaderBasedOnViolation(timeHeadways);
                    } else {
                        ///***** Case Five *****///
                        // We may not switch leadership to another vehicle further downstream because some criteria are not satisfied
                        log.debug("APF found two conditions for assigning leadership further downstream are noy satisfied. Case Five.");
                        log.debug("condition1: " + condition1 + " & condition2: " + condition2);
                        return previousFunctionalLeaderIndex;
                    }
                } else if(closestLowerBoundaryViolation != -1 && closestMaximumSpacingViolation == -1) {
                    // The rest four cases have roughly the same logic: locate the closest violation and assign leadership accordingly
                    ///***** Case Six *****///
                    log.debug("APF found closestLowerBoundaryViolation on partial time headways. Case Six.");
                    return previousFunctionalLeaderIndex - 1 + closestLowerBoundaryViolation;
                } else if(closestLowerBoundaryViolation == -1 && closestMaximumSpacingViolation != -1) {
                    ///***** Case Seven *****///
                    log.debug("APF found closestMaximumSpacingViolation on partial time headways. Case Seven.");
                    return previousFunctionalLeaderIndex + closestMaximumSpacingViolation;
                } else {
                    log.debug("APF found closestMaximumSpacingViolation and closestLowerBoundaryViolation on partial time headways.");
                    if(closestLowerBoundaryViolation > closestMaximumSpacingViolation) {
                        ///***** Case Eight *****///
                        log.debug("closestLowerBoundaryViolation is higher than closestMaximumSpacingViolation on partial time headways. Case Eight.");
                        return previousFunctionalLeaderIndex - 1 + closestLowerBoundaryViolation;
                    } else if(closestLowerBoundaryViolation < closestMaximumSpacingViolation) {
                        ///***** Case Nine *****///
                        log.debug("closestMaximumSpacingViolation is higher than closestLowerBoundaryViolation on partial time headways. Case Nine.");
                        return previousFunctionalLeaderIndex + closestMaximumSpacingViolation;
                    } else {
                        log.error("APF Leader selection parameter is wrong!");
                        return 0;
                    }
                }
            }
        }
    }
    
    // Check if we have enough gap with the front vehicle
    private boolean insufficientGapWithPredecessor(double distanceToFrontVehicle) {
        boolean frontGapIsTooSmall = distanceToFrontVehicle < plugin.getMinGap();
        boolean previousLeaderIsPredecessor = previousFunctionalLeaderID.equals(platoon.get(platoon.size() - 1).staticId);
        boolean frontGapIsNotLargeEnough = distanceToFrontVehicle < plugin.getMaxGap() && previousLeaderIsPredecessor;
        return frontGapIsTooSmall || frontGapIsNotLargeEnough;
    }
    
    // calculate the time headway between every consecutive pair of vehicles
    private double[] calculateTimeHeadway(double[] downtrackDistance, double[] speed) {
        double[] timeHeadways = new double[downtrackDistance.length - 1];
        for(int i = 0; i < timeHeadways.length; i++) {
            if(speed[i + 1] != 0) {
                timeHeadways[i] = (downtrackDistance[i] - downtrackDistance[i + 1]) / speed[i + 1];
            } else {
                timeHeadways[i] = Double.POSITIVE_INFINITY;
            }
        }
        return timeHeadways;
    }
    
    // get the time headway between every consecutive pair of vehicles from start index
    private double[] getTimeHeadwayFromIndex(double[] timeHeadways, int start) {
        return Arrays.copyOfRange(timeHeadways, start, timeHeadways.length);
    }
    
    private int findLowerBoundaryViolationClosestToTheHostVehicle(double[] timeHeadways) {
        for(int i = timeHeadways.length - 1; i >= 0; i--) {
            if(timeHeadways[i] < plugin.getLowerBoundary()) {
                return i;
            }
        }
        return -1;
    }
    
    private int findMaximumSpacingViolationClosestToTheHostVehicle(double[] timeHeadways) {
        for(int i = timeHeadways.length - 1; i >= 0; i--) {
            if(timeHeadways[i] > plugin.getMaxSpacing()) {
                return i;
            }
        }
        return -1;
    }
    
    private int determineLeaderBasedOnViolation(double[] timeHeadways) {
        int closestLowerBoundaryViolation = findLowerBoundaryViolationClosestToTheHostVehicle(timeHeadways);
        int closestMaximumSpacingViolation = findMaximumSpacingViolationClosestToTheHostVehicle(timeHeadways);
        if(closestLowerBoundaryViolation > closestMaximumSpacingViolation) {
            log.debug("APF found violation on closestLowerBoundaryViolation at " + closestLowerBoundaryViolation);
            return closestLowerBoundaryViolation;
        } else if(closestLowerBoundaryViolation < closestMaximumSpacingViolation) {
            log.debug("APF found violation on closestMaximumSpacingViolation at " + closestMaximumSpacingViolation);
            return closestMaximumSpacingViolation + 1;
        } else {
            log.debug("APF found no violations on both closestLowerBoundaryViolation and closestMaximumSpacingViolation");
            return 0;
        }
    }
    
}
