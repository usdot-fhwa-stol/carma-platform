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

package gov.dot.fhwa.saxton.carma.guidance.plugins;

import gov.dot.fhwa.saxton.carma.guidance.trajectory.Trajectory;

/**
 * Interface for plugins which act to provide services to strategic plugins
 * <p>
 * ITacticalPlugin instances do not directly participate in the trajectory planning process
 * but strategic plugins may delegate portions of a trajectory to be planned by a tactical 
 * plugin.
 */
public interface ITacticalPlugin extends IPlugin {
    /**
     * Execute the plugin's planning algorithm on a subset of the specified trajectory
     * 
     * @param traj The trajectory to plan inside
     * @param targetLane The lane ID at the end of the trajectory
     * @param startDistance The distance the maneuver generation should start, m
     * @param startSpeed The speed at the beginning of the trajectory, m/s
     * @param endDistance The distance the maneuver generation should end, m
     * @param endSpeed The speed to be achieved at the end of the trajectory, m/s
     * 
     * @return true if the planning was successful, false otherwise.
     */
    boolean planSubtrajectory(Trajectory traj, int targetLane, double startDistance, double startSpeed,
                              double endDistance, double endSpeed);
}
