/*
 * Copyright (C) 2018-2019 LEIDOS.
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

import gov.dot.fhwa.saxton.carma.guidance.arbitrator.TrajectoryPlanningResponse;
import gov.dot.fhwa.saxton.carma.guidance.trajectory.Trajectory;

/**
 * Interface for Plugins aiming to directly participate in the Trajectory planning process.
 * <p>
 * Nominally IStrategicPlugin implementations are strategic methods for navigating vehicle environments
 * such as Cruising, Platooning, etc.
 */
public interface IStrategicPlugin extends IPlugin {
    /**
     * Execute the plugin's planning algorithm and generate maneuvers in the supplied trajectory if
     * possible.
     * 
     * @param traj The current partially planned Trajectory, which cannot be modified
     * @param expectedEntrySpeed The speed (in m/s) the vehicle is expected to have upon the start of the new trajectory
     */
    TrajectoryPlanningResponse planTrajectory(Trajectory traj, double expectedEntrySpeed);
}
