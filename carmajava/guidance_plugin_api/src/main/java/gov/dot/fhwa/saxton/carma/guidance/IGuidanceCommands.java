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

package gov.dot.fhwa.saxton.carma.guidance;

/**
 * Interface specifying how vehicle commands are to be sent to the object that forward them to the controller driver(s).
 */
public interface IGuidanceCommands {

    /**
     * Sets the desired speed and acceleration limit to be sent to the vehicle controller hardware.
     *
     * @param speed The speed to output
     * @param accel The maximum allowable acceleration in attaining and maintaining that speed
     */
    void setSpeedCommand(double speed, double accel);

    /**
     * Sets the desired axle angle by which to steer the vehicle.
     * <p>
     * Positive angles are interpreted as a command to steer to the right and negative ones as a command to steer
     * to the left.
     * 
     * @param axleAngle The angle in the range [-90.0, 90.0] to command for steering
     * @param lateralAccel The maximum lateral acceleration of the vehicle, in m/s/s
     * @param yawRate The rate of change of the vehicle's heading, in deg/s
     */
    void setSteeringCommand(double axleAngle, double lateralAccel, double yawRate);
}
