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

import java.util.List;

import org.ros.internal.message.Message;

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
     * Callback method to handle mobility requests, responses and operation, which may result in
     * state changing, trajectory replan and information updating.
     * @param request the detailed proposal from other vehicles
     */
    public void onReceiveMobilityMessgae(Message mobilityMessage);
    
    /**
     * Get mobility messages that the current platooning state want to send out
     * The returned message list can only contain operation, request and response
     */
    public List<Message> getNewMobilityOutbound();
    
    // The loop method which is called by the loop method in platooning plug-in
    public void loop() throws InterruptedException;
}
