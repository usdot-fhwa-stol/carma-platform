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
import java.util.SortedSet;
import java.util.TreeSet;

import cav_msgs.NewPlan;
import gov.dot.fhwa.saxton.carma.guidance.util.ILogger;

/**
 * This class manages the changing of platoon list and leader selection process.
 */
public class PlatoonManager implements Runnable {
    
    protected PlatooningPlugin plugin;
    protected SortedSet<PlatoonMember> platoon;
    protected int memberTimeout = 750;
    protected ILogger log;

    public PlatoonManager(PlatooningPlugin plugin, ILogger log) {
        this.plugin = plugin;
        this.log = log;
        // The leader vehicel is always the first on in the set.
        this.platoon = Collections.synchronizedSortedSet(
                new TreeSet<PlatoonMember>((a, b) -> -Double.compare(a.getVehiclePosition(), b.getVehiclePosition())));
        // The max time duration we allow an entry to exist if we did not receive any update from it 
        this.memberTimeout = plugin.messageTimeout;
    }
    
    @Override
    public void run() {
        // This loop will remove any expired entry from platoon list
        while(true) {
            long loopStart = System.currentTimeMillis();
            List<PlatoonMember> removeCandidates = new ArrayList<>();
            int counter = 0;
            for(PlatoonMember pm : platoon) {
                if(System.currentTimeMillis() - pm.getTimestamp() > memberTimeout) {
                    removeCandidates.add(pm);
                }
                counter++;
                log.info("Found vehicel " + pm.getStaticId() + " in platoon list at " + counter);
                log.info(pm.toString());
            }
            if(removeCandidates.size() != 0) {
                for(PlatoonMember candidate: removeCandidates) {
                    log.info("Remove vehicle " + candidate.getStaticId() + " from platoon list because the entry is timeout");
                    platoon.remove(candidate);
                }
            }
            
            long loopEnd = System.currentTimeMillis();
            long sleepDuration = Math.max(memberTimeout - (loopEnd - loopStart), 0);
            try {
                Thread.sleep(sleepDuration);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
    }
    
    /**
     * Given any valid platooning mobility message with staticId, this method
     * tries to find the vehicle member instance and update its status, if a member is not found
     * and that is in the same lane with subject vehicle and in front of the subject vehicle,
     * then we create a vehicle member instance and place it in the correct place in the platoon list.
     * @param plan NewPlan message with valid input string in the format of "CMDSPEED:5.0, DOWNTRACK:100.0, SPEED:5.0"
     */
    protected void memberUpdates(NewPlan plan) {
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
                isExisted = true;
                log.info("Receive and update CACC info on vehicel " + pm.getStaticId());
            }
            if(isExisted) {
                break;
            }
        }
        if(!isExisted) {
            // If we did not find the right entry, we need to consider to add a new one
            // For now, we only add members in front of us
            if(distance > plugin.maneuverInputs.getDistanceFromRouteStart()) {
                PlatoonMember pm = new PlatoonMember(plan.getSenderId(), cmdSpeed, speed, distance, System.currentTimeMillis());
                platoon.add(pm);
                log.info("Add CACC info on new vehicel " + pm.getStaticId());
            }
        }
    }
    
    /**
     * This method contains the logic of leader selection and will return a suitable leader for the subject vehicle
     * TODO Integrate the leader selection algorithm later
     */
    protected PlatoonMember getLeader() {
        return platoon.first();
    }
}
