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

import org.junit.Before;
import org.junit.Test;
import static org.mockito.Mockito.*;

import gov.dot.fhwa.saxton.carma.guidance.plugins.IPlugin;
import gov.dot.fhwa.saxton.carma.guidance.util.ILogger;
import gov.dot.fhwa.saxton.carma.guidance.util.ILoggerFactory;
import gov.dot.fhwa.saxton.carma.guidance.util.LoggerManager;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

public class SlowDownTest {
    
    private FakeGuidanceCommands commands_;
    private IManeuverInputs inputs_;

    @Before
    public void setup() {
        inputs_ = mock(IManeuverInputs.class);
        commands_ = new FakeGuidanceCommands();
        AccStrategyManager.setAccStrategyFactory(new NoOpAccStrategyFactory());
        ILoggerFactory mockFact = mock(ILoggerFactory.class);
        ILogger mockLogger = mock(ILogger.class);
        when(mockFact.createLoggerForClass(any())).thenReturn(mockLogger);
        LoggerManager.setLoggerFactory(mockFact);
    }

    @Test
    public void testSlowDownNominal() {
        //specify the start & end speeds
        double currentSpeed = 10.0; // start at 10 m/s
        double targetSpeed = 5.0; // end at 5 m/s
        final double targetSpeedSqr = targetSpeed * targetSpeed;
        final double maxAccel = 1.0; // 1 m/s^2
        final double responseLag = 0.0; // Assume perfect vehicle response

        double currentDist = 0.0; // Start at 0 m

        when(inputs_.getDistanceFromRouteStart()).thenReturn(currentDist);
        when(inputs_.getCurrentSpeed()).thenReturn(currentSpeed);
        when(inputs_.getMaxAccelLimit()).thenReturn(maxAccel);
        when(inputs_.getResponseLag()).thenReturn(responseLag);

        //plan the maneuver
        SlowDown mvr = new SlowDown(mock(IPlugin.class));
        mvr.setSpeeds(currentSpeed, targetSpeed);
        mvr.setMaxAccel(maxAccel);

        mvr.plan(inputs_, commands_, currentDist);
        double endDist = mvr.getEndDistance();

        //compute expected distance required to perform the maneuver
        double deltaV = targetSpeed - currentSpeed;
        double mvrLength = 1.0*(currentSpeed*deltaV/-maxAccel + 0.5*deltaV*deltaV/-maxAccel) + 0.2*targetSpeed;
        assertEquals(currentDist + mvrLength, endDist, 0.1);

        //execute the maneuver
        boolean done;
        double timeStep = 0.1;
        do {
            // Execute timestep
            done = mvr.executeTimeStep();
            double speedCmd = commands_.getSpeedCmd();
            double accelCmd = commands_.getAccelCmd();
            // Calculate expected result
            double expectedMaxAccel = Math.abs((targetSpeedSqr - currentSpeed * currentSpeed) / (2.0 * (endDist - currentDist)));
            // Check the output commands
            assertEquals(expectedMaxAccel, accelCmd, 0.001); // Max accel should be limited
            assertEquals(targetSpeed, speedCmd, 0.001); // Speed command should not change
            
            // Update our current distance and speed
            currentDist += 0.5 * accelCmd * timeStep + timeStep * currentSpeed;
            currentSpeed = accelCmd * timeStep;
        } while (!done);
    }
}
