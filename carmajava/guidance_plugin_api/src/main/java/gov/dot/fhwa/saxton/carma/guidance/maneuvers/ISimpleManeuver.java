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

import gov.dot.fhwa.saxton.carma.guidance.IGuidanceCommands;

/**
 * Defines an interface to all basic Maneuver (plannable lateral and longitudinal motions) objects.
 */
public interface ISimpleManeuver extends IManeuver {
    /**
     * Plans the maneuver to and makes it ready for execution
     *
     * @param inputs - the object that provides necessary input data about the route
     * @param commands - the object that will take output commands
     * @param startDist - distance from beginning of route at which this maneuver is to begin, m
     * @throws IllegalStateException if required target quantity is not defined prior to this call
     */
    void plan(IManeuverInputs inputs, IGuidanceCommands commands, double startDist) throws IllegalStateException;

    /**
     * Plans the maneuver to a target end distance and makes it ready for execution
     * with the vehicle maxAccel constraints by adjusting the target speed
     * @param inputs - the object that provides necessary input data about the route
     * @param commands - the object that will take output commands
     * @param startDist - distance from beginning of route at which this maneuver is to begin, m
     * @param endDist - the distance from the beginning of route at which this maneuver is to end, m
     * @return adjusted target speed
     * @throws IllegalStateException if required target quantity is not defined prior to this call
     */
    double planToTargetDistance(IManeuverInputs inputs, IGuidanceCommands commands, double startDist, double endDist) throws IllegalStateException;
    
    /**
     * Return if the maneuver can be planned with vehicle lag constraint
     * @param inputs - the object that provides necessary input data about the route
     * @param startDist - distance from beginning of route at which this maneuver is to begin, m
     * @param endDist - the distance from the beginning of route at which this maneuver is to end, m
     * @return boolean
     */
    boolean canPlan(IManeuverInputs inputs, double startDist, double endDist);
}

