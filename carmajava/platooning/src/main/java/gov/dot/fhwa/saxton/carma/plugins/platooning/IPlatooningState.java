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

import cav_msgs.MobilityOperation;
import cav_msgs.MobilityRequest;
import cav_msgs.MobilityResponse;
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
     * Callback method to handle mobility requests which may result in state changing or replan
     * @param request the detailed proposal from other vehicles
     */
    public void onReceiveMobilityRequest(MobilityRequest request);
    
    /**
     * Callback method to handle mobility responses which may determine whether the plugin should execute a plan
     * @param response the negative/positive response from other vehicles
     */
    public void onReceiveMobilityResponse(MobilityResponse response);
    
    /**
     * Callback method to handle mobility operation messages which updates information on the current/surrounding platoon 
     * @param operation the operation from surrounding vehicles
     */
    public void onReceiveMobilityOperation(MobilityOperation operation);
    
    /**
     * Get MobilityOperation messages that the current platooning state want to send out
     */
    public List<MobilityOperation> getNewMobilityOperationOutbound();
    
    /**
     * Get MobilityRequest messages that the current platooning state want to send out
     */
    public MobilityRequest getNewMobilityRequestOutbound();
    
    /**
     * Get MobilityRequest messages that the current platooning state want to send out
     */
    public MobilityResponse getNewMobilityResponseOutbound();
    
    /**
     * Called by the plugin loop method and the purpose is running jobs in the queue in sequence 
     */
    public default void runJobQueue() {
        ///???
    }
}
