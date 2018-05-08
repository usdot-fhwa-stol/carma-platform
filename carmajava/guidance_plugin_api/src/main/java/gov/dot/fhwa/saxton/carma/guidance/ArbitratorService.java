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

package gov.dot.fhwa.saxton.carma.guidance;

import gov.dot.fhwa.saxton.carma.guidance.maneuvers.IManeuver;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.ManeuverType;
import gov.dot.fhwa.saxton.carma.guidance.trajectory.Trajectory;

/**
 * Plugin interface to Arbitrator services provided by the Guidance package
 */
public interface ArbitratorService {
    /**
     * Inform the Arbitrator of trajectory execution failure, necessitating immediate replan and
     * execution of a new trajectory.
     */
    void notifyTrajectoryFailure();

    /**
     * Get the currently executing maneuver of the specified type, null if none such maneuver exists.
     */
    IManeuver getCurrentlyExecutingManeuver(ManeuverType maneuverType);

    /**
     * Get the currently executing Trajectory running on the the host vehicle, null if none is executing
     */
    Trajectory getCurrentTrajectory();

    /**
     * Request the arbitrator to plan and execute a new trajectory.
     */
    void requestNewPlan();

    /**
     * Plan the specified region of the input trajectory as though it were a normal trajectory unto itself.
     * <p>
     * Planning outside startDist and endDist is not allowed. Recursion is only allowed up to a fixed depth
     * at which point planning is considered to have failed. Can only be called by a planning plugin while
     * planning is in progress.
     * 
     * @param startDist the downtrack distance to start planning at, must be within the span of traj, inclusive
     * @param endDist the downtrack distance to end planning at, must be within the span of traj, exclusive
     * 
     * @return The planned trajectory, to be evaluated and inserted into the real Trajectory by the caller.
     * See {@link Trajectory#copyManeuvers} for the recommended way to do this.
     */
    Trajectory planSubtrajectoryRecursively(double startDist, double endDist);
}
