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

import java.time.Duration;
import java.time.LocalDateTime;
import java.util.List;
import java.util.SortedSet;
import java.util.TreeSet;

/**
 * This class manages the changing of platoon list.
 */
public class PlatoonManager {
    
    protected PlatooningPlugin plugin;
    protected volatile LocalDateTime updateTimestamp = LocalDateTime.now();

    public PlatoonManager(PlatooningPlugin plugin) {
        this.plugin = plugin;
    }
    
    /**
     * Given any staticId, this method tries to find the memberId for that vehicle
     * @param staticId the static id of a vehicle
     * @return return the desired member id, if not exist it will return -1
     */
    public int findMemberId(String staticId) {
        SortedSet<PlatoonMember> platoon = this.plugin.platoon;
        for(PlatoonMember member : platoon) {
            if(member.getStaticId().equals(staticId)) {
                return member.getMemberId();
            }
        }
        return -1;
    }
    
    /**
     * Given any staticId, this method tries to find the vehicle member instance
     * @param staticId the static id of a vehicle
     * @return return the desired PlatoonMember instance, if not exist it will return null
     */
    public PlatoonMember findMember(String staticId) {
        SortedSet<PlatoonMember> platoon = this.plugin.platoon;
        for(PlatoonMember member : platoon) {
            if(member.getStaticId().equals(staticId)) {
                return member; 
            }
        }
        return null;
    }
    
    /**
     * When we have a new member join in our platoon, we add its status instance at the end of platoon list 
     * @param member the new member
     */
    public void addNewMember(PlatoonMember member) {
        if(findMemberId(member.getStaticId()) == -1) {
            this.plugin.platoon.add(member);
        }
    }
    
    /**
     * When we have a new member leave our platoon, we need to remove it from the platoon list
     * and update the memberId for vehicles after that
     * @param staticId
     */
    public void deleteMember(String staticId) {
        PlatoonMember deleteCandidate = findMember(staticId);
        if(deleteCandidate != null) {
            this.plugin.platoon.remove(deleteCandidate);
            int updateStart = deleteCandidate.getMemberId();
            for(PlatoonMember member : this.plugin.platoon) {
                if(member.getMemberId() > updateStart) {
                    member.setMemberId(member.getMemberId() - 1);
                }
            }
        }
    }
    
    /**
     * This method is used in cases where a waypoint disallows the capability to empty platoon list
     */
    public void disablePlatooning() {
        // TODO we may need to disband only temporarily to later reform
        this.plugin.platoon = new TreeSet<PlatoonMember>();
    }
    
    /**
     * Update the platoon member status when we receive their status
     * @param status
     */
    public void updateMemberStatus(List<PlatoonMember> status) {
        // TODO update member status based on STATUS messages
        updateTimestamp = LocalDateTime.now();
    }
    
    /**
     * Get elapsed time between the current time and the latest update timestamp 
     * @return timeElapsed in milliseconds
     */
    public long getTimeSinceLastUpdate() {
        LocalDateTime now = LocalDateTime.now();
        long timeElapsed = Duration.between(updateTimestamp, now).toMillis();
        return timeElapsed;
    }
}
