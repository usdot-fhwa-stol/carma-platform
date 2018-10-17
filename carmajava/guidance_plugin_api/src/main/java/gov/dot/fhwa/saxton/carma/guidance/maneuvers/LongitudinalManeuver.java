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
import gov.dot.fhwa.saxton.carma.guidance.plugins.IPlugin;

/**
 * Base class for all longitudinal maneuvers, providing the adaptive cruise control (ACC) functionality.
 */
public abstract class LongitudinalManeuver extends ManeuverBase {

    protected double maxAccel_ = 0.999; // m/s^2 absolute value; default is a conservative value
    protected final double SMALL_SPEED_CHANGE = 2.5; // m/s
    protected final IAccStrategy accStrategy;
    protected boolean completed = false;
    protected long startTime_ = 0;
    protected double startSpeed_ = -1.0; // m/s
    protected double endSpeed_ = -1.0; // m/s
    protected double workingAccel_; // m/s^2 that we will actually use
    protected static final double SPEED_EPSILON = 0.0001;

    public LongitudinalManeuver(IPlugin planner) {
        super(planner);
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
        // TODO Evaluate removal of verify location
        // GPS drift can cause a stationary vehicle to fail to pass the start point of a maneuver
        // The verifyLocation check is removed to prevent this
        // verifyLocation();

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
            commands_.setSpeedCommand(executeSpeedCommand, accStrategy.getMaxAccel());
        } else {
            commands_.setSpeedCommand(executeSpeedCommand, workingAccel_);
        }
        return completed;
    }

        /**
     * Stores the beginning and target speed of the maneuver.
     * Since maneuvers will generally be chained together during planning, this is the only way that a maneuver
     * can know what speed the vehicle will have after completing its predecessor maneuver.
     * @param startSpeed - the expected speed at the beginning of the maneuver, m/s
     * @param targetSpeed - target speed at end of maneuver, m/s
     */
    public void setSpeeds(double startSpeed, double targetSpeed) {
        startSpeed_ = startSpeed;
        endSpeed_ = targetSpeed;
    }

    /**
     * Returns the specified starting speed for the maneuver.  To be used for longitudinal maneuvers only.
     * @return m/s
     */
    public double getStartSpeed() {
        return startSpeed_;
    }

    /**
     * Returns the specified target speed for the end of the maneuver.  To be used for longitudinal maneuvers only.
     * @return m/s
     */
    public double getTargetSpeed() {
        return endSpeed_;
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

    @Override
    public String toString() {
        return "LongitudinalManeuver [startSpeed_=" + startSpeed_ + ", endSpeed_=" + endSpeed_ + ", startDist_=" + startDist_ + ", endDist_=" + endDist_ + "]";
    }
    
}
