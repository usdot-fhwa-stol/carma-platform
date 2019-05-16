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

package gov.dot.fhwa.saxton.carma.guidance.signals;

import java.util.Optional;

/**
 * A deadband filter.
 * If error between signal and setpoint is < deadband then filter will return the setpoint instead
 * Otherwise passes the provided signal through unchanged
 */
public class Deadband implements Filter<Double> {
    private double setpoint = 0;
    private double deadband = 0;

    /**
     * Construct a PID controller with the specified Kp, Ki, Kd, and setpoint values
     * 
     * @param setpoint The initial setpoint to be used for error computation
     */
    public Deadband(double setpoint, double deadband) {
        this.setpoint = setpoint;
        this.deadband = deadband;
    }

    /**
     * Change the setpoint
     * 
     * @param setpoint The setpoint
     */
    public void adjustSetPoint(double setpoint) {
        this.setpoint = setpoint;
    }

    /**
     * Change the deadband
     * 
     * @param deadband The deadband
     */
    public void adjustDeadband(double deadband) {
        this.deadband = deadband;
    }

    /**
     * Apply the Deadband filter
     * </p>
     * If signal is within deadband of setpoint then the setpoint is returned otherwise the signal is returned
     * 
     * @param signal The measured plant value
     * @return An Optional always containing the control response value
     */
    public Optional<Signal<Double>> apply(Signal<Double> signal) {
        double error = Math.abs(setpoint - signal.getData());
        if (error < deadband) {
            return Optional.of(new Signal<>(setpoint, signal.getTimestamp()));
        }
        return Optional.of(signal);
    }

    @Override public void reset() {
        // No maintained state to reset
    }

    @Override
    public void changeSetpoint(Double setpoint) {
        // No setpoint in this filter
    }
}
