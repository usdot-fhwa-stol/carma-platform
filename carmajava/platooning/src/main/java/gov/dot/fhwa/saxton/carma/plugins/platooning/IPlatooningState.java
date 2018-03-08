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

import cav_msgs.MobilityIntro;
import cav_msgs.NewPlan;
import gov.dot.fhwa.saxton.carma.guidance.arbitrator.TrajectoryPlanningResponse;
import gov.dot.fhwa.saxton.carma.guidance.trajectory.Trajectory;

public interface IPlatooningState {
    
    /**
     * Execute the plugin's planning algorithm and generate maneuvers in the supplied trajectory if possible.
     * @param traj The current partially planned Trajectory, which cannot be modified
     * @param expectedEntrySpeed The speed (in m/s) the vehicle is expected to have upon the start of the new trajectory
     * @return Trajectory planning response
     */
    public TrajectoryPlanningResponse planTrajectory(Trajectory traj, double expectedEntrySpeed);
    
    /**
     * Callback method to handle negotiation requests which may result in state changing
     * @param plan the detailed negotiation proposal from another vehicle
     */
    public void onReceiveNegotiationMessage(NewPlan plan);
    
    /**
     * Get the MobilityIntro message the current state want to send out
     */
    public MobilityIntro getNewOutboundIntroMessage();
    
    /**
     * Called by the plugin loop method and the purpose is checking if plugin needs to change state 
     */
    public void checkCurrentState();
}
