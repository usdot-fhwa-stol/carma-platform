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
    
    public int findMemberId(String staticId) {
        SortedSet<PlatoonMember> platoon = this.plugin.platoon;
        for(PlatoonMember member : platoon) {
            if(member.getStaticId().equals(staticId)) {
                return member.getMemberId();
            }
        }
        return -1;
    }
    
    public PlatoonMember findMember(String staticId) {
        SortedSet<PlatoonMember> platoon = this.plugin.platoon;
        for(PlatoonMember member : platoon) {
            if(member.getStaticId().equals(staticId)) {
                return member; 
            }
        }
        return null;
    }
    
    public void addNewMember(PlatoonMember member) {
        if(findMemberId(member.getStaticId()) == -1) {
            this.plugin.platoon.add(member);
        }
    }
    
    public void deleteMember(String staticId) {
        PlatoonMember deleteCandidate = findMember(staticId);
        if(deleteCandidate != null) {
            this.plugin.platoon.remove(deleteCandidate);
        }
    }
    
    public void disablePlatooning() {
        this.plugin.platoon = new TreeSet<PlatoonMember>();
    }
    
    public void updateMemberStatus(List<PlatoonMember> status) {
        // TODO update member status based on STATUS messages
        updateTimestamp = LocalDateTime.now();
    }
    
    public long getTimeSinceLastUpdate() {
        LocalDateTime now = LocalDateTime.now();
        long timeElapsed = Duration.between(updateTimestamp, now).toMillis();
        return timeElapsed;
    }
}
