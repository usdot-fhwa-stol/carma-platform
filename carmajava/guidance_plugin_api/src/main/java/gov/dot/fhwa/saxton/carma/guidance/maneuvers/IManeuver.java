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

package gov.dot.fhwa.saxton.carma.guidance.maneuvers;

/**
 * Defines an interface to all Maneuver objects.
 */
public interface IManeuver {
    /**
     * Executes a single time step of the maneuver that has already been planned, by calculating the
     * instantaneous commands then passing those commands to the vehicle's controller driver.  There is no
     * assumption made about the duration or uniformity of time steps.
     *
     * ASSUMES that the plan method has already run to completion - there is no check for this condition!
     *
     * @return true if the maneuver has completed; false if it is still in progress
     * @throws IllegalStateException if called when vehicle's position along route is not between the
     * maneuver's start and end distances
     */
    boolean executeTimeStep() throws IllegalStateException;

    /**
     * Retrieves the specified start distance of the maneuver.
     * @return distance from beginning of the route at which the maneuver begins, m
     */
    double getStartDistance();


    /**
     * Retrieves the calculated end distance of the maneuver.
     * @return distance from beginning of the route at which the maneuver is to complete, m
     */
    double getEndDistance();
}
