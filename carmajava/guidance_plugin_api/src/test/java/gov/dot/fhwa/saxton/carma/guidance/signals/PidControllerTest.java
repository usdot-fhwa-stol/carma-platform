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

import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import java.util.List;
import java.util.Optional;

import static org.junit.Assert.*;
import static org.mockito.Mockito.*;

public class PidControllerTest {

    @Before
    public void setUp() {
        setpoint = 1.0;
        controller = new PidController(Kp, Ki, Kd, setpoint);
    }

    @Test
    public void testOneTimestep() {
        Optional<Signal<Double>> out = controller.apply(new Signal<>(0.0));
        double error = setpoint - 0.0;
        double pTerm = error * Kp;

        assertTrue(out.isPresent());
        assertEquals(out.get().getData(), pTerm * error, EPSILON);
    }

    @Test
    public void testTwoTimesteps() {
        // First timestep
        Signal<Double> s1 = new Signal<>(0.0);
        Optional<Signal<Double>> out1 = controller.apply(s1);
        double error1 = setpoint - s1.getData();
        double pTerm1 = error1 * Kp;

        // Second timestep
        Signal<Double> s2 = new Signal<>(0.0);
        Optional<Signal<Double>> out2 = controller.apply(s2);
        double error2 = setpoint - s2.getData();
        double dt = s2.getTimestamp() - s1.getTimestamp();
        double pTerm2 = error2 * Kp;
        double iTerm2 = dt * error2;
        double dTerm2 = (error2 - error1) / dt;

        assertTrue(out2.isPresent());
        assertEquals(out2.get().getData(), pTerm2 + iTerm2 - dTerm2, EPSILON);
    }

    private PidController controller;
    private double setpoint = 1.0;
    private static final double Kp = 1.0;
    private static final double Ki = 0.01;
    private static final double Kd = 0.001;
    private static final double EPSILON = 0.001;
}
