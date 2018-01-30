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

package gov.dot.fhwa.saxton.carma.guidance.maneuvers;

import gov.dot.fhwa.saxton.carma.guidance.IGuidanceCommands;

/**
 * Lane change maneuver. Moves the vehicle from the current lane to the target lane
 */
public class LaneChange extends LateralManeuver {

    protected int targetLane_ = 0; // 0 indexed from right to left
    protected double RIGHT_LANE_CHANGE = 1.0;
    protected double LEFT_LANE_CHANGE = -1.0;
    protected double KEEP_LANE = 0.0;

    public LaneChange(String plannerName) {
        super(plannerName);
    }

    @Override
    public void plan(IManeuverInputs inputs, IGuidanceCommands commands, double startDist)
            throws IllegalStateException {
        super.plan(inputs, commands, startDist);
        throw new IllegalStateException("Cannot plan a lane change maneuver without specifying start and end distance");
    }

    @Override
    public double planToTargetDistance(IManeuverInputs inputs, IGuidanceCommands commands, double startDist,
            double endDist) throws IllegalStateException, ArithmeticException {
        endDist_ = endDist;
        return super.planToTargetDistance(inputs, commands, startDist, endDist);
    }

    @Override
    protected double getAxleAngleCmd() {
        // TODO provide a real implementation of this
        if (inputs_.getCurrentLane() < targetLane_) { // Target lane is to the left
            return LEFT_LANE_CHANGE;
        } else if (inputs_.getCurrentLane() > targetLane_) { // Target lane is on the right
            return RIGHT_LANE_CHANGE;
        } else {
            return KEEP_LANE;
        }
    }

    /**
     * Stores the target lane ID, to be used for lateral maneuvers only.
     * @param targetLane - target lane number at end of maneuver
     */
    public void setTargetLane(int targetLane) {
        targetLane_ = targetLane;
    }

    /**
     * Returns true if a lane change maneuver can be planned over this distance
     */
    @Override
    public boolean canPlan(IManeuverInputs inputs, double startDist, double endDist) {

        //assume we will be going the current speed
        //typical lane change takes about 4 seconds
        double distRequired = 4.0 * inputs.getCurrentSpeed();
        return (endDist - startDist) >= distRequired;
    }
}
