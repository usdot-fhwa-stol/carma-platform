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
import java.util.Collections;
import java.util.List;

import cav_msgs.NewPlan;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.IManeuverInputs;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.ManeuverInputs;
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
                Collections.sort(platoon, (a, b) -> (Double.compare(a.getVehiclePosition(), b.getVehiclePosition())));
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
                Collections.sort(platoon, (a, b) -> (Double.compare(a.getVehiclePosition(), b.getVehiclePosition())));
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
        PlatoonMember leader = null;
        if(platoon.isEmpty()) {
            // This will make sure we only apply leader selection algorithm when the platoon list is not empty
            return leader;
        } else {
            // return the first vehicle in the platoon as default if no valid algorithm is indicated
            leader = platoon.get(0);
        }
        if(plugin.getAlgorithmType() == 1) {
            PlatoonMember newLeader = allPredecessorFollowing();
            leader = newLeader;
            previousLeader = leader == null ? "" : leader.getStaticId();
        }
        return leader;
    }
    
    //If this method returns null, it means it assume itself as the leader
    private PlatoonMember allPredecessorFollowing() {
        PlatoonMember newLeader = platoon.get(0);
        IManeuverInputs inputs = this.psl.getManeuverPlanner().getManeuverInputs();
        // if the distance headway between the subject vehicle and its predecessor is an issue, it should follow its predecessor
        if(inputs.getDistanceToFrontVehicle() < plugin.getMinGap() ||
                (inputs.getDistanceToFrontVehicle() < plugin.getMaxGap() && previousLeader.equals(platoon.get(platoon.size() - 1).getStaticId()))) {
            newLeader = platoon.get(platoon.size() - 1);
        } else {
            // calculate the time headway between every consecutive pair of vehicles
            double[] timeHeadways = new double[platoon.size()];
            for(int i = 0; i < timeHeadways.length; i++) {
                if(i < timeHeadways.length - 1) {
                    timeHeadways[i] = (platoon.get(i).getVehiclePosition() - platoon.get(i + 1).getVehiclePosition()) / platoon.get(i + 1).getVehicleSpeed(); 
                } else {
                    timeHeadways[i] = inputs.getDistanceToFrontVehicle() / inputs.getCurrentSpeed();
                }
            }
            int closestLowerBoundaryViolation = -1, closestMaximumSpacingViolation = -1;
            if(previousLeader.equals(platoon.get(0).getStaticId())) {
                for(int i = timeHeadways.length - 1; i >= 0; i++) {
                    if(timeHeadways[i] < plugin.getLowerBoundary()) {
                        closestLowerBoundaryViolation = i;
                        break;
                    }
                }
                for(int i = timeHeadways.length - 1; i >= 0; i++) {
                    if(timeHeadways[i] > plugin.getMaxSpacing()) {
                        closestMaximumSpacingViolation = i;
                        break;
                    }
                }
                if(closestLowerBoundaryViolation == -1 && closestMaximumSpacingViolation == -1) {
                    return newLeader;
                } else if(closestLowerBoundaryViolation != -1 && closestMaximumSpacingViolation == -1) {
                    newLeader = platoon.get(closestLowerBoundaryViolation);
                } else if(closestLowerBoundaryViolation == -1 && closestMaximumSpacingViolation != -1) {
                    int leaderIndex = closestMaximumSpacingViolation + 1;
                    newLeader = leaderIndex > platoon.size() - 1 ? null : platoon.get(leaderIndex);
                } else {
                    if(closestLowerBoundaryViolation > closestMaximumSpacingViolation) {
                        newLeader = platoon.get(closestLowerBoundaryViolation);
                    } else if(closestLowerBoundaryViolation < closestMaximumSpacingViolation) {
                        int leaderIndex = closestMaximumSpacingViolation + 1;
                        newLeader = leaderIndex > platoon.size() - 1 ? null : platoon.get(leaderIndex);
                    } else {
                        log.error("APF Leader selection cannot handle this case 1");
                    }
                }
            } else if(!previousLeader.equals("")) {
                int indexOfPreviousLeader = -1;
                for(int i = 0; i < platoon.size(); i++) {
                    if(platoon.get(i).getStaticId().equals(previousLeader)) {
                        indexOfPreviousLeader = i;
                        break;
                    }
                }
                if(indexOfPreviousLeader == -1) {
                    log.error("APF Leader selection cannot handle this case 2");
                    return newLeader;
                }
                double[] temporary_time_headways = new double[platoon.size() - indexOfPreviousLeader];
                for(int i = 0; i < temporary_time_headways.length; i++) {
                    temporary_time_headways[i] = timeHeadways[indexOfPreviousLeader + i];
                }
                for(int i = temporary_time_headways.length - 1; i >= 0; i++) {
                    if(temporary_time_headways[i] < plugin.getLowerBoundary()) {
                        closestLowerBoundaryViolation = i;
                        break;
                    }
                }
                for(int i = temporary_time_headways.length - 1; i >= 0; i++) {
                    if(temporary_time_headways[i] > plugin.getMaxSpacing()) {
                        closestMaximumSpacingViolation = i;
                        break;
                    }
                }
                if(closestLowerBoundaryViolation == -1 && closestMaximumSpacingViolation == -1) {
                    if(timeHeadways[indexOfPreviousLeader] < plugin.getUpperBoundary() ||
                            timeHeadways[indexOfPreviousLeader - 1] > plugin.getMinSpacing()) {
                        newLeader = platoon.get(indexOfPreviousLeader);
                    } else if(timeHeadways[indexOfPreviousLeader] > plugin.getUpperBoundary() &&
                            timeHeadways[indexOfPreviousLeader - 1] < plugin.getMinSpacing()) {
                        closestLowerBoundaryViolation = -1;
                        closestMaximumSpacingViolation = -1;
                        for(int i = timeHeadways.length - 1; i >= 0; i++) {
                            if(timeHeadways[i] < plugin.getLowerBoundary()) {
                                closestLowerBoundaryViolation = i;
                                break;
                            }
                        }
                        for(int i = timeHeadways.length - 1; i >= 0; i++) {
                            if(timeHeadways[i] > plugin.getMaxSpacing()) {
                                closestMaximumSpacingViolation = i;
                                break;
                            }
                        }
                        if(closestLowerBoundaryViolation == -1 && closestMaximumSpacingViolation == -1) {
                            newLeader = platoon.get(0);
                        } else if(closestLowerBoundaryViolation != -1 && closestMaximumSpacingViolation == -1) {
                            newLeader = platoon.get(closestLowerBoundaryViolation);
                        } else if(closestLowerBoundaryViolation == -1 && closestMaximumSpacingViolation != -1) {
                            int leaderIndex = closestMaximumSpacingViolation + 1; 
                            newLeader = leaderIndex > platoon.size() - 1 ? null : platoon.get(leaderIndex);
                        } else {
                            if(closestLowerBoundaryViolation > closestMaximumSpacingViolation) {
                                newLeader = platoon.get(closestLowerBoundaryViolation);
                            } else if(closestLowerBoundaryViolation < closestMaximumSpacingViolation) {
                                int leaderIndex = closestMaximumSpacingViolation + 1;
                                newLeader = leaderIndex > platoon.size() - 1 ? null : platoon.get(leaderIndex);
                            } else {
                                log.error("APF Leader selection cannot handle this case 3");
                            }
                        }
                    }
                    // Make sure thoese else if are correct.......
                    else if(closestLowerBoundaryViolation != -1 && closestMaximumSpacingViolation == -1) {
                        newLeader = platoon.get(indexOfPreviousLeader - 1 + closestLowerBoundaryViolation);
                    } else if(closestLowerBoundaryViolation == -1 && closestMaximumSpacingViolation != -1) {
                        newLeader = platoon.get(indexOfPreviousLeader + closestMaximumSpacingViolation);
                    } else {
                        if(closestLowerBoundaryViolation > closestMaximumSpacingViolation) {
                            newLeader = platoon.get(indexOfPreviousLeader - 1 + closestLowerBoundaryViolation);
                        } else if(closestLowerBoundaryViolation < closestMaximumSpacingViolation) {
                            newLeader = platoon.get(indexOfPreviousLeader + closestMaximumSpacingViolation);
                        } else {
                            log.error("APF Leader selection cannot handle this case 4");
                        }
                    }
                }
            }
        }
        return newLeader;
        
    }
    
    protected int getPlatooningSize() {
        return platoon.size();
    }
}
