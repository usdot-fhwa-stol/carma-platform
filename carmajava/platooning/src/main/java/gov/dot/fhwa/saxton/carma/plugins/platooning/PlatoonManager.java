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

import cav_msgs.NewPlan;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.IManeuverInputs;
import gov.dot.fhwa.saxton.carma.guidance.plugins.PluginServiceLocator;
import gov.dot.fhwa.saxton.carma.guidance.util.ILogger;

/**
 * This class manages the changing of platoon list and leader selection process.
 */
public class PlatoonManager implements Runnable {
    
    protected PlatooningPlugin plugin;
    protected List<PlatoonMember> platoon;
    protected ILogger log;
    protected PluginServiceLocator psl;
    protected String previousLeader = "";
    protected int indexOfPreviousLeader = -1;

    public PlatoonManager(PlatooningPlugin plugin, ILogger log, PluginServiceLocator psl) {
        this.plugin = plugin;
        this.log = log;
        this.psl = psl;
        // The leader vehicle is always the first on in the list.
        this.platoon = Collections.synchronizedList(new ArrayList<>());
    }
    
    @Override
    public void run() {
        try {
            while(!Thread.currentThread().isInterrupted()) {
                long loopStart = System.currentTimeMillis();
                removeExpiredMember();
                long loopEnd = System.currentTimeMillis();
                long sleepDuration = Math.max(plugin.messageTimeout - (loopEnd - loopStart), 0);
                Thread.sleep(sleepDuration);
            }
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
        
    }
    
    /**
     * Given any valid platooning mobility message with staticId, this method
     * tries to find the vehicle member instance and update its status, if a member is not found
     * and that is in the same lane with subject vehicle and in front of the subject vehicle,
     * then we create a vehicle member instance and place it in the correct place in the platoon list.
     * @param plan NewPlan message with valid input string in the format of "CMDSPEED:5.0, DOWNTRACK:100.0, SPEED:5.0"
     */
    protected synchronized void memberUpdates(NewPlan plan) {
        String vehicleId = plan.getSenderId();
        String[] inputsArray = plan.getInputs().split(",");
        boolean isExisted = false;
        double cmdSpeed = Double.parseDouble(inputsArray[0].split(":")[1]);
        double distance = Double.parseDouble(inputsArray[1].split(":")[1]);
        double speed = Double.parseDouble(inputsArray[2].split(":")[1]);
        for(PlatoonMember pm : platoon) {
            if(pm.getStaticId().equals(vehicleId)) {
                pm.setCommandSpeed(cmdSpeed);
                pm.setVehiclePosition(distance);
                pm.setVehicleSpeed(speed);
                pm.setTimestamp(System.currentTimeMillis());
                log.info("Receive and update CACC info on vehicel " + pm.getStaticId());
                log.info("    Speed = " + pm.getVehicleSpeed());
                log.info("    Location = " + pm.getVehiclePosition());
                log.info("    CommandSpeed = " + pm.getCommandSpeed());
                isExisted = true;
                break;
            }
        }
        if(!isExisted) {
            // If we did not find the right entry, we need to consider to add a new one
            // For now, we only add members in front of us
            if(distance > plugin.getManeuverInputs().getDistanceFromRouteStart()) {
                PlatoonMember pm = new PlatoonMember(plan.getSenderId(), cmdSpeed, speed, distance, System.currentTimeMillis());
                platoon.add(pm);
                Collections.sort(platoon, (a, b) -> (Double.compare(b.getVehiclePosition(), a.getVehiclePosition())));
                log.info("Add CACC info on new vehicle " + pm.getStaticId());
            } else {
                log.info("Ignore new vehicle info because it is behind us. Its id is " + plan.getSenderId());
            }
        }
    }
    
    // This method removes any expired entry from platoon list
    protected synchronized void removeExpiredMember() {
        List<PlatoonMember> removeCandidates = new ArrayList<>();
        int counter = 0;
        for(PlatoonMember pm : platoon) {
            if(System.currentTimeMillis() - pm.getTimestamp() > plugin.messageTimeout) {
                removeCandidates.add(pm);
            }
            counter++;
            log.debug("Found vehicel " + pm.getStaticId() + " in platoon list at " + counter);
            log.debug(pm.toString());
        }
        if(removeCandidates.size() != 0) {
            for(PlatoonMember candidate: removeCandidates) {
                log.info("Remove vehicle " + candidate.getStaticId() +
                        " from platoon list because the entry is timeout by " + (System.currentTimeMillis() - candidate.getTimestamp()) + " ms");
                platoon.remove(candidate);
            }
        }
    }
    
    /**
     * This method contains will use the indicated algorithm to determine
     * which vehicle in the platoon will function as the leader.
     * CommandGenerator will use the output of this function as the baseline cmd_speed 
     */
    protected synchronized PlatoonMember getLeader() {
        PlatoonMember newLeader = null;
        if(platoon.isEmpty()) {
            // This will make sure we only apply leader selection algorithm when the platoon list is not empty
            return newLeader;
        } else {
            // return the first vehicle in the platoon as default if no valid algorithm is indicated
            newLeader = platoon.get(0);
            // We should not update the previous leader id here because that is not the final choice
            //previousLeader = newLeader.getStaticId();
        }
        if(plugin.getAlgorithmType() == 1) {
            int newLeaderIndex = allPredecessorFollowing();
            newLeader = newLeaderIndex >= platoon.size() ? null : platoon.get(newLeaderIndex);
            indexOfPreviousLeader = newLeaderIndex >= platoon.size() ? -1 : newLeaderIndex;
            previousLeader = newLeader == null ? "" : newLeader.getStaticId();
        }
        return newLeader;
    }
    
    // If this method returns the size of platoon, it means the subject vehicle should follow his own commands
    // Otherwise, it will return the index of the leader in the platoon list
    private int allPredecessorFollowing() {
        int result = 0;
        // If we do not have any leader in the previous time step
        if(previousLeader.equals("")) {
            ///***** Case One *****///
            log.debug("APF algorithm did not found a leader in previous time step. Case one!");
            log.debug("APF returns the first one in this platoon as the leader. Case one!");
            return result;
        }
        IManeuverInputs inputs = this.psl.getManeuverPlanner().getManeuverInputs();
        // Generate an array of downtrack distance for every vehicles in this platoon including the host vehicle
        double[] downtrackDistance = new double[platoon.size() + 1];
        for(int i = 0; i < platoon.size(); i++) {
            downtrackDistance[i] = platoon.get(i).getVehiclePosition(); 
        }
        downtrackDistance[downtrackDistance.length - 1] = inputs.getDistanceFromRouteStart();
        // Generate an array of speed for every vehicles in this platoon including the host vehicle
        double[] speed = new double[platoon.size() + 1];
        for(int i = 0; i < platoon.size(); i++) {
            speed[i] = platoon.get(i).getVehicleSpeed();
        }
        speed[speed.length - 1] = inputs.getCurrentSpeed();
        // if the distance headway between the subject vehicle and its predecessor is an issue, it should follow its predecessor
        if(insufficientGapWithPredecessor(inputs.getDistanceToFrontVehicle())) {
            ///***** Case Two *****///
            log.debug("APF algorithm decide there is an issue with the gap with predecessor. Case Two!");
            log.debug("APF returns the predecessor as the leader. Case Two!");
            result = platoon.size() - 1;
        } else {
            // implementation of the regular APF algorithm
            double[] timeHeadways = calculateTimeHeadway(downtrackDistance, speed);
            log.debug("APF calculate time headways: " + Arrays.toString(timeHeadways));
            log.debug("APF found the previous leader is " + indexOfPreviousLeader);
            int closestLowerBoundaryViolation, closestMaximumSpacingViolation;
            // if the previous leader is the first vehicle in the platoon
            if(indexOfPreviousLeader == 0) {
                result = determineLeaderBasedOnViolation(timeHeadways);
                if(result == 0) {
                    ///***** Case Zero *****///
                    // This should be the most regular case
                    log.debug("APF did not found violations on lower boundary or maximum spacing. Case Zero.");
                    log.debug("APF decides to continue follow the first vehicle.");
                } else {
                    ///***** Case Three *****///
                    log.debug("APF found violations on lower boundary or maximum spacing. Case Three.");
                    log.debug("APF decide " + result + " as the leader. Case Three!");
                }
            } else {
                // if the previous leader is not the first one
                double[] temporaryTimeHeadways = calculateTimeHeadwayFromIndex(timeHeadways, indexOfPreviousLeader);
                closestLowerBoundaryViolation = findLowerBoundaryViolationClosestToTheHostVehicle(temporaryTimeHeadways);
                closestMaximumSpacingViolation = findMaximumSpacingViolationClosestToTheHostVehicle(temporaryTimeHeadways);
                // if there is no violations in time headways
                if(closestLowerBoundaryViolation == -1 && closestMaximumSpacingViolation == -1) {
                    // Two conditions for assigning leadership further downstream
                    boolean condition1 = timeHeadways[indexOfPreviousLeader] > plugin.getUpperBoundary();
                    boolean condition2 = timeHeadways[indexOfPreviousLeader - 1] < plugin.getMinSpacing();
                    if(condition1 && condition2) {
                        ///***** Case Four *****///
                        log.debug("APF found two conditions for assigning leadership further downstream are satisfied. Case Four.");
                        log.debug("APF decide " + result + " as the leader based on possible violations on all time headways. Case Four!");
                        result = determineLeaderBasedOnViolation(timeHeadways);
                        //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!change here!!!!!!!!!!!!!!!!!!
                    } else {
                        ///***** Case Five *****///
                        log.debug("APF found two conditions for assigning leadership further downstream are noy satisfied. Case Five.");
                        log.debug("APF returns the previous leader: " + indexOfPreviousLeader + ". Case Five.");
                        result = indexOfPreviousLeader;
                    }
                } else if(closestLowerBoundaryViolation != -1 && closestMaximumSpacingViolation == -1) {
                    ///***** Case Six *****///
                    log.debug("APF found closestLowerBoundaryViolation on partial time headways. Case Six.");
                    result = indexOfPreviousLeader - 1 + closestLowerBoundaryViolation;
                    log.debug("APF decides to assign leader further upstream" + result + ". Case Six.");
                } else if(closestLowerBoundaryViolation == -1 && closestMaximumSpacingViolation != -1) {
                    ///***** Case Seven *****///
                    log.debug("APF found closestMaximumSpacingViolation on partial time headways. Case Seven.");
                    result = indexOfPreviousLeader + closestMaximumSpacingViolation;
                    log.debug("APF decides to assign leader further upstream" + result + ". Case Seven.");
                } else {
                    log.debug("APF found closestMaximumSpacingViolation and closestLowerBoundaryViolation on partial time headways.");
                    if(closestLowerBoundaryViolation > closestMaximumSpacingViolation) {
                        ///***** Case Eight *****///
                        log.debug("closestLowerBoundaryViolation is higher than closestMaximumSpacingViolation on partial time headways. Case Eight.");
                        result = indexOfPreviousLeader - 1 + closestLowerBoundaryViolation;
                        log.debug("APF decides to assign leader further upstream" + result + ". Case Eight.");
                    } else if(closestLowerBoundaryViolation < closestMaximumSpacingViolation) {
                        ///***** Case Nine *****///
                        log.debug("closestMaximumSpacingViolation is higher than closestLowerBoundaryViolation on partial time headways. Case Nine.");
                        result = indexOfPreviousLeader + closestMaximumSpacingViolation;
                        log.debug("APF decides to assign leader further upstream" + result + ". Case Nine.");
                    } else {
                        log.error("APF Leader selection cannot handle this case.");
                        log.error("APF decides to assign the first vehicle as the leader by default.");
                        result = 0;
                    }
                }
            }
        }
        return result;
    }
    
    // Check if we have enough gap with the front vehicle
    private boolean insufficientGapWithPredecessor(double distanceToFrontVehicle) {
        boolean frontGapIsTooSmall = distanceToFrontVehicle < plugin.getMinGap();
        boolean previousLeaderIsPredecessor = previousLeader.equals(platoon.get(platoon.size() - 1).getStaticId());
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
    
    // calculate the time headway between every consecutive pair of vehicles from start index
    private double[] calculateTimeHeadwayFromIndex(double[] timeHeadways, int start) {
        return Arrays.stream(timeHeadways).skip(start).toArray();
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
            return closestLowerBoundaryViolation;
        } else if(closestLowerBoundaryViolation < closestMaximumSpacingViolation) {
            return closestMaximumSpacingViolation + 1;
        } else {
            return 0;  
        }
    }
    
    protected int getPlatooningSize() {
        return platoon.size();
    }
}
