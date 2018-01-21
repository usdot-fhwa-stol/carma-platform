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
     * Execute the plugin's planning algorithm on a subset of the specified trajectory, inserting a FutureManeuver
     * into the trajectory between the indicated locations.
     * 
     * @param traj The trajectory to plan inside
     * @param startDistance The distance the maneuver generation should start, m
     * @param endDistance The distance the maneuver generation should end, m
     *
     * @return true if the planning was successful, false otherwise.  Note that a return of false may indicate
     * that a future maneuver is being negotiated, or it may indicate a planning failure so that the trajectory
     * is not modified at all.
     */
    boolean planSubtrajectory(Trajectory traj, double startDistance, double endDistance);
}
