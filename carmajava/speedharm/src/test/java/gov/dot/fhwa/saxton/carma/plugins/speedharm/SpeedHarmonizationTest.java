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

package gov.dot.fhwa.saxton.carma.plugins.speedharm;

import gov.dot.fhwa.saxton.carma.guidance.maneuvers.AccStrategyManager;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.IAccStrategy;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.IAccStrategyFactory;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.NoOpAccStrategy;
import gov.dot.fhwa.saxton.carma.guidance.util.ILogger;
import gov.dot.fhwa.saxton.carma.guidance.util.ILoggerFactory;
import gov.dot.fhwa.saxton.carma.guidance.util.LoggerManager;
import org.junit.Before;
import org.junit.Test;
import org.ros.message.Duration;
import org.ros.message.Time;

import static junit.framework.Assert.assertEquals;
import static junit.framework.Assert.assertTrue;
import static junit.framework.Assert.fail;
import static org.junit.Assert.assertFalse;
import static org.mockito.ArgumentMatchers.anyObject;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

public class SpeedHarmonizationTest {

  private FakeManeuverInputForSpeedHarm inputs_;
  private FakeGuidanceCommands commands_;
  private IAccStrategy accStrategy_;

  @Before public void setup() {
    ILoggerFactory mockFact = mock(ILoggerFactory.class);
    ILogger mockLogger = mock(ILogger.class);
    when(mockFact.createLoggerForClass(anyObject())).thenReturn(mockLogger);
    LoggerManager.setLoggerFactory(mockFact);
    inputs_ = new FakeManeuverInputForSpeedHarm();
    commands_ = new FakeGuidanceCommands();
    accStrategy_ = new NoOpAccStrategy();
  }

  @Test public void testInputValidation() {

    FakeSpeedHarmInputs shInputs = new FakeSpeedHarmInputs(10.0, 2.5);

    // Check valid configuration with distance and time
    try {
      SpeedHarmonizationManeuver shManeuver =
        new SpeedHarmonizationManeuver(null, shInputs, inputs_, commands_, accStrategy_, 10, 20, Time.fromMillis(1000),
          Time.fromMillis(2000));
    } catch (IllegalArgumentException e) {
      fail("Valid speed harm maneuver failed to construct");
    }

    // Check invalid distances
    try {
      SpeedHarmonizationManeuver shManeuver =
        new SpeedHarmonizationManeuver(null, shInputs, inputs_, commands_, accStrategy_, 10, 5, Time.fromMillis(1000),
          Time.fromMillis(2000));
      fail("Invalid speed harm maneuver constructed");
    } catch (IllegalArgumentException e) {
    }

    // Check invalid times
    try {
      SpeedHarmonizationManeuver shManeuver =
        new SpeedHarmonizationManeuver(null, shInputs, inputs_, commands_, accStrategy_, 10, 5, Time.fromMillis(2000),
          Time.fromMillis(1000));
      fail("Invalid speed harm maneuver constructed");
    } catch (IllegalArgumentException e) {
    }

    // Check valid configuration with distance and speed
    try {
      SpeedHarmonizationManeuver shManeuver =
        new SpeedHarmonizationManeuver(null, shInputs, inputs_, commands_, accStrategy_, 10, 20, 10, 20);
    } catch (IllegalArgumentException e) {
      fail("Valid speed harm maneuver failed to construct");
    }

    // Check invalid speeds
    try {
      SpeedHarmonizationManeuver shManeuver =
        new SpeedHarmonizationManeuver(null, shInputs, inputs_, commands_, accStrategy_, 10, 5, 20, 10);
      fail("Invalid speed harm maneuver constructed");
    } catch (IllegalArgumentException e) {
    }

    // Check valid configuration with all inputs
    try {
      SpeedHarmonizationManeuver shManeuver =
        new SpeedHarmonizationManeuver(null, shInputs, inputs_, commands_, accStrategy_, 10, 20, Time.fromMillis(1000),
          Time.fromMillis(2000), 6, 9);
    } catch (IllegalArgumentException e) {
      fail("Valid speed harm maneuver failed to construct");
    }

    // Check invalid configuration with all inputs
    try {
      SpeedHarmonizationManeuver shManeuver =
        new SpeedHarmonizationManeuver(null, shInputs, inputs_, commands_, accStrategy_, 10, 20, Time.fromMillis(1000),
          Time.fromMillis(2000), 3, 4);
      fail("Invalid speed harm maneuver constructed");
    } catch (IllegalArgumentException e) {
    }

    // Check invalid configuration with all inputs
    try {
      SpeedHarmonizationManeuver shManeuver =
        new SpeedHarmonizationManeuver(null, shInputs, inputs_, commands_, accStrategy_, 10, 20, Time.fromMillis(100),
          Time.fromMillis(2000), 1000, 2000);
      fail("Invalid speed harm maneuver constructed");
    } catch (IllegalArgumentException e) {
    }

  }

  @Test public void testExecuteTimestep() {
    FakeSpeedHarmInputs shInputs = new FakeSpeedHarmInputs(10.0, 2.5);
    SpeedHarmonizationManeuver shManeuver;
    // Check valid configuration with distance and time
    try {
      shManeuver = new SpeedHarmonizationManeuver(null, shInputs, inputs_, commands_, accStrategy_, 10, 20, Time.fromMillis(1000),
        Time.fromMillis(2000));
      shManeuver.setTimeout(Duration.fromMillis(2000));
    } catch (IllegalArgumentException e) {
      fail("Valid speed harm maneuver failed to construct");
      return;
    }
    shManeuver.setMaxAccel(2.5);

    shInputs.setMaxAccelLimit(1.0);
    shInputs.setSpeedCommand(3.0);
    assertFalse(shManeuver.executeTimeStep());
    assertEquals(3.0, commands_.getSpeedCmd(), 0.0001);
    assertEquals(1.0, commands_.getAccelCmd(), 0.0001);

    // Check negative acceleration
    shInputs.setMaxAccelLimit(-4.0);
    try {
      shManeuver.executeTimeStep();
      fail("Negative maximum acceleration not caught by maneuver");
    } catch (IllegalStateException e) {
    }

    // Check truncation of max accel
    shInputs.setMaxAccelLimit(4.0);
    assertFalse(shManeuver.executeTimeStep());
    assertEquals(3.0, commands_.getSpeedCmd(), 0.0001);
    assertEquals(2.5, commands_.getAccelCmd(), 0.0001);

    // Check timeout
    shInputs.setTimeSinceLastUpdate(Duration.fromMillis(3000));
    try {
      shManeuver.executeTimeStep();
      fail("Timeout was not caught when executing timestep");
    } catch (IllegalStateException e) {
    }

    shInputs.setTimeSinceLastUpdate(Duration.fromMillis(100));

    // Check maneuver completion
    inputs_.setDistanceFromRouteStart(30);
    assertTrue(shManeuver.executeTimeStep());

  }
}
