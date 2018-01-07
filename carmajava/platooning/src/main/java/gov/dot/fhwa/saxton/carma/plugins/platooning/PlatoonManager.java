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

import java.util.LinkedList;
import java.util.List;

public class PlatoonManager {
    
    protected PlatooningPlugin plugin;

    public PlatoonManager(PlatooningPlugin plugin) {
        this.plugin = plugin;
    }
    
    public int findMemberId(String staticId) {
        List<PlatoonMember> platoon = this.plugin.platoon;
        for(int i = 0; i < platoon.size(); i++) {
            if(platoon.get(i).getStaticId().equals(staticId)) {
                return i;
            }
        }
        return -1;
    }
    
    public void addNewMember(PlatoonMember member) {
        if(findMemberId(member.getStaticId()) == -1) {
            this.plugin.platoon.add(member);
        }
    }
    
    public void deleteMember(String staticId) {
        int deleteCandidate = findMemberId(staticId);
        if(deleteCandidate >= 0) {
            this.plugin.platoon.remove(deleteCandidate);
        }
    }
    
    public void disablePlatooning() {
        this.plugin.platoon = new LinkedList<PlatoonMember>();
    }
}
