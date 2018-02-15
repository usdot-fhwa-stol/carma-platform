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

/**
 * This class manages the changing of platoon list.
 */
public class PlatoonManager implements Runnable {
    
    protected PlatooningPlugin plugin;
    protected List<PlatoonMember> platoon = Collections.synchronizedList(new ArrayList<>());
    protected int memberTimeout = 750;

    public PlatoonManager(PlatooningPlugin plugin) {
        this.plugin = plugin;
        // The max time duration we allow an entry to exist if we did not receive any update from it 
        this.memberTimeout = plugin.messageTimeout;
    }
    
    @Override
    public void run() {
        // This loop will remove any expired entry from platoon list
        while(true) {
            long loopStart = System.currentTimeMillis();
            List<PlatoonMember> removeCandidates = new ArrayList<>();
            for(PlatoonMember pm : platoon) {
                if(System.currentTimeMillis() - pm.getTimestamp() > memberTimeout) {
                    removeCandidates.add(pm);
                }
            }
            if(removeCandidates.size() != 0) {
                for(PlatoonMember candidate: removeCandidates) {
                    platoon.remove(candidate);
                }
                // Update member id in the platoon
                for(int i = 0; i < platoon.size(); i++) {
                    platoon.get(i).setMemberId(i);
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
        for(PlatoonMember pm : platoon) {
            if(pm.getStaticId().equals(vehicleId)) {
                pm.setCommandSpeed(Double.parseDouble(inputsArray[0].split(":")[1]));
                pm.setVehiclePosition(Double.parseDouble(inputsArray[1].split(":")[1]));
                pm.setVehicleSpeed(Double.parseDouble(inputsArray[2].split(":")[1]));
                pm.setTimestamp(System.currentTimeMillis());
                isExisted = true;
            }
            if(isExisted) {
                break;
            }
        }
        if(!isExisted) {
            //TODO did not find the entry, need to add a new one
        }
    }
    
}
