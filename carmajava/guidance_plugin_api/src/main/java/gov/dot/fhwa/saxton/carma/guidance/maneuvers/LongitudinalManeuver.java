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
 * Base class for all longitudinal maneuvers, providing the adaptive cruise control (ACC) functionality.
 */
public abstract class LongitudinalManeuver extends ManeuverBase {

    protected double startSpeed_ = -1.0; // m/s
    protected double endSpeed_ = -1.0; // m/s
    protected double maxAccel_ = 0.999; // m/s^2 absolute value; default is a conservative value
    protected final double SMALL_SPEED_CHANGE = 2.5; // m/s
    protected final IAccStrategy accStrategy;
    protected boolean completed = false;
    protected long startTime_ = 0;
    protected double workingAccel_; // m/s^2 that we will actually use
    protected static final double SPEED_EPSILON = 0.0001;

    public LongitudinalManeuver() {
        this.accStrategy = AccStrategyManager.newAccStrategy();
    }

    @Override
    public void plan(IManeuverInputs inputs, IGuidanceCommands commands, double startDist)
            throws IllegalStateException {
        super.plan(inputs, commands, startDist);

        //check that speeds have been defined
        if (startSpeed_ < -0.5 || endSpeed_ < -0.5) {
            throw new IllegalStateException(
                    "Longitudinal maneuver plan attempted without previously defining the start/target speeds.");
        }
    }

    @Override
    public double planToTargetDistance(IManeuverInputs inputs, IGuidanceCommands commands, double startDist,
            double endDist) throws IllegalStateException, ArithmeticException {
        return super.planToTargetDistance(inputs, commands, startDist, endDist);
    }

    @Override
    public boolean executeTimeStep() {
        verifyLocation();

        if (startTime_ == 0) {
            startTime_ = System.currentTimeMillis();
        }

        double speedCmd = generateSpeedCommand();
        double distToFrontVehicle = inputs_.getDistanceToFrontVehicle();
        double currentSpeed = inputs_.getCurrentSpeed();
        double frontVehicleSpeed = inputs_.getFrontVehicleSpeed();

        double overrideCmd = accStrategy.computeAccOverrideSpeed(distToFrontVehicle, frontVehicleSpeed, currentSpeed, speedCmd);
        boolean overrideActive = Math.abs(speedCmd - overrideCmd) > SPEED_EPSILON;

        if (overrideActive) {
            log_.warn(String.format("ACC override engaged! Speed command reduced from %.02f m/s to %.02f m/s. Adjusting max accel to command %.02f m/s^2",
            speedCmd,
            overrideCmd,
            accStrategy.getMaxAccel()));
        }

        executeSpeedCommand(overrideCmd, overrideActive);

        return true;
    }

    public abstract double generateSpeedCommand();

    public boolean executeSpeedCommand(double executeSpeedCommand, boolean overrideActive) {
        //send the command to the vehicle
        if (overrideActive) {
            commands_.setCommand(executeSpeedCommand, accStrategy.getMaxAccel());
        } else {
            commands_.setCommand(executeSpeedCommand, workingAccel_);
        }
        return completed;
    }

    @Override
    public void setSpeeds(double startSpeed, double targetSpeed) throws UnsupportedOperationException {
        startSpeed_ = startSpeed;
        endSpeed_ = targetSpeed;
    }

    @Override
    public double getStartSpeed() {
        return startSpeed_;
    }

    @Override
    public double getTargetSpeed() {
        return endSpeed_;
    }

    @Override
    public void setTargetLane(int targetLane) throws UnsupportedOperationException {
        throw new UnsupportedOperationException("Attempting to use setTargetLane on a longitudinal maneuver.");
    }

    /**
     * Specifies the maximum acceleration allowed in the maneuver.  Note that this value will apply to both speeding
     * up and slowing down (symmetrical).
     * @param limit - max (absolute value) allowed, m/s^2
     */
    public void setMaxAccel(double limit) {
        if (limit > 0.0) { //can't be equal to zero
            maxAccel_ = limit;
        }
    }
}
