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
 * Base class for all lateral maneuvers.
 */
public abstract class LaneChange extends LateralManeuver {

    protected boolean completed = false;
    protected long startTime_ = 0;
    protected double maxAxleAngleRate = 0.0; 
    protected double maxAccel_ = 0.0;
    protected int targetLane_ = 0; // 0 indexed from right to left
    protected int startingLane_ = 0;
    protected double axleAngle_ = 0.0; // rad: Angle in radians to turn the wheels. Positive is left, Negative is right
    protected double lateralAccel_ = 0.0; // Max acceleration which can be caused by a turn
    protected double yawRate_ = 0.0;  // rad/s: Max axel angle velocity
    protected double RIGHT_LANE_CHANGE = 1.0;
    protected double LEFT_LANE_CHANGE = -1.0;
    protected double KEEP_LANE = 0.0;
    protected double timePerLaneChange_ = 4.0; //s


    @Override
    public void plan(IManeuverInputs inputs, IGuidanceCommands commands, double startDist)
            throws IllegalStateException {
        super.plan(inputs, commands, startDist);

        double deltaV = endSpeed_ - startSpeed_;
        double numberOfLanes = Math.abs(targetLane_ - startingLane_);
        double deltaT = timePerLaneChange_ * numberOfLanes;
        endDist_ = deltaT * ((deltaV / 2.0) + startSpeed_) + startDist;
    }

    @Override
    public double planToTargetDistance(IManeuverInputs inputs, IGuidanceCommands commands, double startDist,
            double endDist) throws IllegalStateException, ArithmeticException {
        throw new IllegalStateException("Cannot plan a lane change maneuver without specifying start and end speeds");
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
     * Stores the starting lane ID, to be used for lateral maneuvers only.
     * @param startingLane - lane number at the start
     */
    public void setStartingLane(int startingLane) {
        startingLane_ = startingLane;
    }

    /**
     * Stores the starting lane ID, to be used for lateral maneuvers only.
     * @param timePerLaneChange - The amount of time in sec that it takes to perform one lane change
     */
    public void setTimePerLaneChange(int timePerLaneChange) {
        timePerLaneChange_ = timePerLaneChange;
    }
}
