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
 * Represents a longitudinal maneuver in which the vehicle steadily increases its speed.
 */
public class SpeedUp extends LongitudinalManeuver {
    private double                  deltaT_;                    // expected duration of the "ideal" speed change, sec
    private static final double MIN_ACCEL_ = 0.1;
    public SpeedUp(IPlugin planner) {
        super(planner);
    }

    /**
     * ASSUMES that the target speed has been specified such that it does not exceed and infrastructure speed limit.
     */
    @Override
    public void plan(IManeuverInputs inputs, IGuidanceCommands commands, double startDist) throws IllegalStateException, ArithmeticException {
        super.plan(inputs, commands, startDist);
        // TODO update planning for new speed method
        //verify proper speed relationships
        if (endSpeed_ <= startSpeed_) {
            log_.error("Speedup.plan called with startSpeed = " + startSpeed_ + " , endSpeed = " + endSpeed_ + ". Throwing exception.");
            throw new ArithmeticException("SpeedUp maneuver being planned with startSpeed = " + startSpeed_ +
                                            ", endSpeed = " + endSpeed_);
        }

        //if speed change is going to be only slight then
        double deltaV = endSpeed_ - startSpeed_; //always positive
        workingAccel_ = maxAccel_;
        if (deltaV < SMALL_SPEED_CHANGE) {
            //cut the acceleration rate to half the limit
            workingAccel_ = 0.5 * maxAccel_;
        }

        //compute the distance to be covered during a linear (in time) speed change, assuming perfect vehicle response
        double idealLength = (startSpeed_*deltaV + 0.5*deltaV*deltaV) / workingAccel_;

        //compute the time it will take to perform this ideal speed change
        deltaT_ = deltaV / workingAccel_;

        //add the distance covered by the expected vehicle lag plus a little buffer to cover time step discretization
        double lagDistance = startSpeed_*inputs_.getResponseLag();
        endDist_ = startDist_ + idealLength + lagDistance + 0.2*endSpeed_;
        log_.debug("SpeedUp.plan completed with deltaV = " + deltaV + ", idealLength = " + idealLength + ", deltaT = " + deltaT_
                    + ", endDist = " + endDist_);
    }
    
    @Override
    public double planToTargetDistance(IManeuverInputs inputs, IGuidanceCommands commands, double startDist, double endDist) {
        super.planToTargetDistance(inputs, commands, startDist, endDist);
        // TODO update planning for new speed method
        //verify proper speed and distance relationships
        if (endSpeed_ <= startSpeed_) {
            log_.error("SpeedUp.planToTargetDistance entered with startSpeed = " + startSpeed_ + ", endSpeed = " + endSpeed_ + ". Throwing exception.");
            throw new ArithmeticException("SpeedUp maneuver being planned with startSpeed = " + startSpeed_ + ", endSpeed = " + endSpeed_);
        }
        if (endDist <= startDist) {
            log_.error("SpeedUp.planToTargetDistance entered with startDist = " + startDist + ", endDist = " + endDist + ". Throwing exception.");
            throw new ArithmeticException("SpeedUp maneuver being planned with startDist = " + startDist + ", endDist = " + endDist);
        }

        double deltaV = endSpeed_ - startSpeed_; //always positive
        double lagDistance = startSpeed_ * inputs_.getResponseLag();
        double rawDisplacement = endDist - startDist;
        double displacement = rawDisplacement - lagDistance;
        if(displacement <= 0) {
            log_.error("SpeedUp.planToTargetDistance do not have enough distance to plan when accounting for lag time." +
                " Maneuver will be planned without this consideration. StartDist: " + startDist + " endDist: " + endDist + " lagDistance: " + lagDistance);
            displacement = rawDisplacement;
        }
        workingAccel_ = (startSpeed_ * deltaV + 0.5 * deltaV * deltaV) / displacement;
        if (workingAccel_ > maxAccel_) {
            workingAccel_ = maxAccel_;
            // Solve v^2 - v_0^2 = (2 * a * D) for v, where v_0 is startSpeed_, a is maxAccel_ and D is displacement
            endSpeed_ = Math.sqrt(startSpeed_ * startSpeed_ + 2 * workingAccel_ * displacement);
            deltaV = endSpeed_ - startSpeed_;
            log_.warn("SpeedUp.plantoTargetDistance attempting to use illegal workingAccel. Adjusting target speed to a reasonable value: " + endSpeed_);
        }

        //compute the time it will take to perform this ideal speed change
        deltaT_ = deltaV / workingAccel_;
        endDist_ = endDist;
        log_.debug("SpeedUp.planToTargetDistance complete with endDist = " + endDist_ + ", deltaT = " + deltaT_ + ", targetSpeed = " + endSpeed_);
        return endSpeed_;
    }
    
    @Override
    public boolean canPlan(IManeuverInputs inputs, double startDist, double endDist) {
        double displacement = endDist - startDist - (inputs.getResponseLag() * startSpeed_); 
        return displacement > 0;
    }
    
    @Override
    public double generateSpeedCommand() throws IllegalStateException {
        // The target speed will always be our ending speed but the working acceleration will be adjusted for smooth approach. 
        // TODO make api clearer for workingAccel_ It is not intuitive we are doing this

        double currentSpeed = inputs_.getCurrentSpeed();
        double currentSpeedSqr = currentSpeed * currentSpeed;
        double targetSpeedSqr = endSpeed_ * endSpeed_;
        double remainingDistance = endDist_ - inputs_.getDistanceFromRouteStart();
        // If we have slightly passed the end point just continue as before
        if (remainingDistance < 0) {
            return endSpeed_;
        }

        double neededAccel = Math.abs((targetSpeedSqr - currentSpeedSqr) / (2 * remainingDistance));

        if (neededAccel > inputs_.getMaxAccelLimit()) {
          workingAccel_ = inputs_.getMaxAccelLimit();
        } else if (neededAccel > MIN_ACCEL_) {
          workingAccel_ = neededAccel;
        } else {
            workingAccel_ = MIN_ACCEL_;
        }

        return endSpeed_; // Always target our ending speed

    }

    
}
