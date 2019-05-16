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

package gov.dot.fhwa.saxton.carma.guidance.maneuvers;

import gov.dot.fhwa.saxton.carma.guidance.IGuidanceCommands;
import gov.dot.fhwa.saxton.carma.guidance.plugins.IPlugin;

/**
 * Base class for all lateral maneuvers.
 */
public abstract class LateralManeuver extends ManeuverBase {

    protected boolean completed = false;
    protected long startTime_ = 0;
    protected double maxAxleAngleRate = 0.0; 
    protected double axleAngle_ = 0.0; // rad: Angle in radians to turn the wheels. Positive is left, Negative is right
    protected double lateralAccel_ = 0.0; // Max acceleration which can be caused by a turn
    protected double yawRate_ = 0.0;  // rad/s: Max axel angle velocity
    protected int endingRelativeLane_ = 0; // 0 is no change +1 is moving left -1 is moving right

    public LateralManeuver(IPlugin planner, int endingRelativeLane) {
        super(planner);
        endingRelativeLane_ = endingRelativeLane;
    }

    @Override
    public void plan(IManeuverInputs inputs, IGuidanceCommands commands, double startDist)
            throws IllegalStateException {
        super.plan(inputs, commands, startDist);  
    }

    @Override
    public double planToTargetDistance(IManeuverInputs inputs, IGuidanceCommands commands, double startDist,
            double endDist) throws IllegalStateException, ArithmeticException {
        if (endDist <= startDist) {
            log_.error("planToTargetDistance entered with startDist = " + startDist + ", endDist = " + endDist + ". Throwing exception.");
            throw new ArithmeticException("Lateral maneuver being planned with startDist = " + startDist + ", endDist = " + endDist);
        }
        return super.planToTargetDistance(inputs, commands, startDist, endDist);
    }

    @Override
    public boolean executeTimeStep() {
        // TODO Evaluate removal of verify location
        // GPS drift can cause a stationary vehicle to fail to pass the start point of a maneuver
        // The verifyLocation check is removed to prevent this
        // verifyLocation();

        if (startTime_ == 0) {
            startTime_ = System.currentTimeMillis();
        }

        //TODO Decide if more functionality or validation is needed here
        axleAngle_ = getAxleAngleCmd();
        commands_.setSteeringCommand(axleAngle_, lateralAccel_, yawRate_);
        return true;
    }

    /**
     * Returns the axle angle cmd to execute for the next timestep
     */
    protected abstract double getAxleAngleCmd();

    @Override
    public abstract boolean canPlan(IManeuverInputs inputs, double startDist, double endDist);

    /**
     * Sets the acceleration constraints for this maneuver 
     *
     * @param yawRate Max axel angle velocity (How fast the wheel can turn)
     * @param lateralAccel Max acceleration which can be caused by a turn
     */
    public void setDynamicLimits(double yawRate, double lateralAccel) {
        yawRate_ = yawRate;
        lateralAccel_ = lateralAccel;
    }

    public int getEndingRelativeLane() {
        return this.endingRelativeLane_;
    }
}
