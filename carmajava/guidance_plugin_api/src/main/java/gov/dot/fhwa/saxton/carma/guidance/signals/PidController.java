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

package gov.dot.fhwa.saxton.carma.guidance.signals;

import java.util.Optional;

/**
 * PID controller implementation as a signal.
 * 
 * Transforms a plant value into a control signal for correcting error relative
 * to the PID controller's configured setpoint.
 */
public class PidController implements Filter<Double> {
    private final double Kp;
    private final double Ki;
    private final double Kd;
    private double setpoint = 0;
    private double integrator = 0;
    private double integratorMax = Double.POSITIVE_INFINITY;
    private Optional<Signal<Double>> lastError = Optional.empty();


    /**
     * Construct a PID controller with the specified Kp, Ki, Kd, and setpoint values
     * 
     * @param Kp The coefficient associated with the proportional response of the PID controller
     * @param Ki The coefficient associated with the integral response of the PID controller
     * @param Kd The coefficient associated with the derivative response of the PID controller
     * @param setpoint The initial setpoint to be used for error computation
     */
    public PidController(double Kp, double Ki, double Kd, double setpoint) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.setpoint = setpoint;
    }

    /**
     * Change the setpoint of this PID controller
     * 
     * @param setpoint The setpoint to try to attain
     */
    @Override
    public void changeSetpoint(Double setpoint) {
        this.setpoint = setpoint;
    }

    /**
     * Apply the PID controller to the input signal
     * </p>
     * The input signal is treated as the plant value and compared to the setpoint configured
     * with adjustSetPoint for the purpose of error computation. The output signal is the 
     * control response to correct the error.
     * 
     * @param signal The measured plant value
     * @return An Optional always containing the control response value
     */
    public Optional<Signal<Double>> apply(Signal<Double> signal) {
        double error = signal.getData() - setpoint;
        double output = Kp * error;

        // If this isn't our first timestep, handle the I and D terms
        if (lastError.isPresent()) {
            double dt = signal.getTimestamp() - lastError.get().getTimestamp();

            if (Ki > 0) {
                integrator += Math.min(error * dt, integratorMax);
                output += Ki * integrator;
            }

            if (Kd > 0) {
                output -= Kd * (error - lastError.get().getData()) / dt;
            }
        }

        lastError = Optional.of(new Signal<>(error, signal.getTimestamp()));

        return Optional.of(new Signal<>(output, signal.getTimestamp()));
    }

    @Override public void reset() {
        integrator = 0;
        lastError = Optional.empty();
    }
}
