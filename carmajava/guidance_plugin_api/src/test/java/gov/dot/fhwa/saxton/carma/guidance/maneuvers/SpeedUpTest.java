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

import org.junit.Before;
import org.junit.Test;

import static junit.framework.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
import static org.mockito.Mockito.*;

import gov.dot.fhwa.saxton.carma.guidance.util.ILogger;
import gov.dot.fhwa.saxton.carma.guidance.util.ILoggerFactory;
import gov.dot.fhwa.saxton.carma.guidance.util.LoggerManager;

public class SpeedUpTest {
    private FakeManeuverInputs   inputs_;
    private FakeGuidanceCommands commands_;


    @Before
    public void setup() {
        inputs_ = new FakeManeuverInputs();
        commands_ = new FakeGuidanceCommands();
        AccStrategyManager.setAccStrategyFactory(new NoOpAccStrategyFactory());
        ILoggerFactory mockFact = mock(ILoggerFactory.class);
        ILogger mockLogger = mock(ILogger.class);
        when(mockFact.createLoggerForClass(anyObject())).thenReturn(mockLogger);
        LoggerManager.setLoggerFactory(mockFact);
    }

    @Test
    public void testSpeedUpNominal() {
        //specify the start & end speeds
        double targetSpeed = inputs_.getTargetSpeed();
        double maxAccel = inputs_.getAccel();

        //plan the maneuver
        SpeedUp mvr = new SpeedUp();
        double startSpeed = inputs_.getSlowSpeed();
        mvr.setSpeeds(startSpeed, targetSpeed);
        mvr.setMaxAccel(maxAccel);

        double startDist = inputs_.getStartDist();
        mvr.plan(inputs_, commands_, startDist);
        double endDist = mvr.getEndDistance();

        //compute expected distance required to perform the maneuver
        double deltaV = targetSpeed - startSpeed;
        double mvrLength = 1.0*(startSpeed*deltaV/maxAccel + 0.5*deltaV*deltaV/maxAccel) + 0.2*targetSpeed;
        assertEquals(startDist + mvrLength, endDist, 0.1);

        //execute the maneuver
        inputs_.setTestCase("SpeedUpNominal");
        double prevCmd = 0.0;
        boolean done;
        int i = 0;
        do {
            done = mvr.executeTimeStep();
            double speedCmd = commands_.getSpeedCmd();
            double accelCmd = commands_.getAccelCmd();
            assertEquals(inputs_.getAccel(), accelCmd, 0.001);

            double expectedSpeedCmd = Math.min(inputs_.getSlowSpeed() + (double)i * 0.1 * accelCmd, inputs_.getTargetSpeed());
            System.out.println("expected = " + expectedSpeedCmd + ", actual = " + speedCmd + ", prev = " + prevCmd);
            assertTrue(speedCmd >= prevCmd);
            assertEquals(expectedSpeedCmd, speedCmd, 0.02);
            prevCmd = speedCmd;
            ++i;
        } while (!done);
    }
}
