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


public class PlatoonPlan {

    // This enum describes the status of a platooning plan
    protected enum PlanStatus {
        WAITING_ON_RESPONSE,
        EXECUTING_PLAN,
    }
    
    // This enum describes all possible plan type of a platooning plan
    protected enum PlatooningPlanType {
        JOIN_FROM_REAR,
        CANDIDATE_FOLLOWER_JOIN,
        LOOKING_FOR_MEMBERS,
        PLATOONING_STATUS
    }
    
    protected long                 planStartTime;
    protected String               planId;
    protected PlanStatus           status;
    protected PlatooningPlanType   type;
    
    public PlatoonPlan(long planStartTime, String planId, PlanStatus status, PlatooningPlanType type) {
        this.planStartTime = planStartTime;
        this.planId        = planId;
        this.status        = status;
        this.type          = type;
    }
    
}