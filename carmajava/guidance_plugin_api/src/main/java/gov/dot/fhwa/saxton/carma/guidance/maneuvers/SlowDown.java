/*
 * TODO: Copyright (C) 2017 LEIDOS.
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
 * Represents a longitudinal maneuver in which the vehicle steadily decreases its speed.
 */
public class SlowDown extends LongitudinalManeuver {

    private double                  workingAccel_;              // m/s^2 that we will actually use
    private double                  deltaT_;                    // expected duration of the "ideal" speed change, sec
    private long                    startTime_ = 0;             // time that the maneuver execution started, ms


    @Override
    public void planToTargetSpeed(IManeuverInputs inputs, IGuidanceCommands commands, double startDist) throws IllegalStateException, ArithmeticException {
        super.planToTargetSpeed(inputs, commands, startDist);

        //verify proper speed relationships
        if (endSpeed_ >= startSpeed_) {
            throw new ArithmeticException("SlowDown maneuver being planned with startSpeed = " + startSpeed_ +
                    ", endSpeed = " + endSpeed_);
        }

        //if speed change is going to be only slight then
        double deltaV = startSpeed_ - endSpeed_; //always positive
        workingAccel_ = maxAccel_;
        if (deltaV < SMALL_SPEED_CHANGE) {
            //cut the acceleration rate to half the limit
            workingAccel_ = 0.5 * maxAccel_;
        }

        //compute the distance to be covered during a linear (in time) speed change, assuming perfect vehicle response
        double idealLength = (startSpeed_*deltaV + 0.5*deltaV*deltaV) / workingAccel_;

        //compute the time it will take to perform this ideal speed change
        deltaT_ = deltaV / workingAccel_;

        //add the distance covered by the expected vehicle lag and account for a little settling buffer
        double lagDistance = startSpeed_*inputs_.getResponseLag();
        endDist_ = startDist_ + idealLength + lagDistance + 0.2*endSpeed_;
   }


    @Override
    public void planToTargetDistance(IManeuverInputs inputs, IGuidanceCommands commands, double startDist, double endDist) {
        super.planToTargetDistance(inputs, commands, startDist, endDist);

        //verify proper speed relationships
        if (endSpeed_ >= startSpeed_) {
            throw new ArithmeticException("SlowDown maneuver being planned with startSpeed = " + startSpeed_ +
                                            ", endSpeed = " + endSpeed_);
        }

        //if speed change is going to be only slight then
        double deltaV = endSpeed_ - startSpeed_; //always positive
        double displacement = endDist - startDist;
        workingAccel_ = (startSpeed_ * deltaV + 0.5 * deltaV * deltaV) / displacement;

        //compute the time it will take to perform this ideal speed change
        deltaT_ = deltaV / workingAccel_;

        endDist_ = endDist;
    }


    @Override
    public boolean executeTimeStep() throws IllegalStateException {
        boolean completed = false;

        verifyLocation();

        if (startTime_ == 0) {
            startTime_ = System.currentTimeMillis();
        }

        //compute command based on linear interpolation on time
        //Note that commands will begin changing immediately, although the actual speed will not change much until
        // the response lag has passed. Thus, we will hit the target speed command sooner than we pass the end distance.
        double currentTime = System.currentTimeMillis();
        double factor = 0.001 * (double)(currentTime - startTime_) / deltaT_;
        if (factor > 1.0) {
            factor = 1.0;
            completed = true;
        }
        double cmd = startSpeed_ + factor*(endSpeed_ - startSpeed_);

        //invoke the ACC override
        cmd = accOverride(cmd);

        //send the command to the vehicle
        commands_.setCommand(cmd, workingAccel_);
        return completed;
    }
}
