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
import java.util.Queue;
import java.util.concurrent.ConcurrentLinkedQueue;

/**
 * Simple MovingAverageFilter
 * A moving average filter serves as extermely simple low pass filter
 */
public class MovingAverageFilter implements Filter<Double> {
    private Queue<Double> samples = new ConcurrentLinkedQueue<Double>();
    private int numSamples = 1; 
    private double sum = 0;

    /**
     * Construct a PID controller with the specified Kp, Ki, Kd, and setpoint values
     * 
     * @param Kp The coefficient associated with the proportional response of the PID controller
     * @param Ki The coefficient associated with the integral response of the PID controller
     * @param Kd The coefficient associated with the derivative response of the PID controller
     * @param setpoint The initial setpoint to be used for error computation
     */
    public MovingAverageFilter(int numSamples) {
        this.numSamples = numSamples;
    }

    @Override
    public void changeSetpoint(Double setpoint) {
        // No setpoint in this filter
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
        Double newSample = signal.getData();
        samples.add(newSample); // Add the sample to the queue of samples

        Double oldestValue = 0.0;
        if (samples.size() > numSamples) {
            oldestValue = samples.poll(); // Pop the oldest sample from the queue
        }

        sum += -oldestValue + newSample;
        Double result = sum / samples.size();

        return Optional.of(new Signal<>(result, signal.getTimestamp()));
    }

    @Override public void reset() {
        sum = 0;
        samples = new ConcurrentLinkedQueue<Double>();
    }
}
