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

import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import java.util.List;

import static org.junit.Assert.*;
import static org.mockito.Mockito.*;

public class BasicAccStrategyTest {
  private BasicAccStrategy bas;
  private static final double EPSILON = 0.001;

  private boolean fpEquals(double a, double b, double epsilon) {
    return Math.abs(b - a) <= epsilon;
  }

  @Test
  public void testNoOverride() {
    assertFalse(bas.evaluateAccTriggerConditions(20.0, 10.0, 10.0));
    assertFalse(bas.evaluateAccTriggerConditions(11.0, 10.0, 10.0));

    assertTrue(fpEquals(10.0, bas.computeAccOverrideSpeed(20.0, 15.0, 15.0, 10.0), EPSILON));
  }

  @Test
  public void testHalfOverride() {
    double frontVehicleSpeed = 10.0;
    double currentSpeed = 20.0;
    double cmdOverride = bas.computeAccOverrideSpeed(15.0, frontVehicleSpeed, currentSpeed, 10.0);
    assertEquals(15.0, cmdOverride, 0.001);
  }

  @Test
  public void testFullOverride() {
    double frontVehicleSpeed = 10.0;
    double currentSpeed = 20.0;
    double cmdOverride = bas.computeAccOverrideSpeed(5.0, frontVehicleSpeed, currentSpeed, 10.0);
    assertEquals(10.0, cmdOverride, 0.001);
  }

  @Test
  public void testTooCloseOverride() {
    double frontVehicleSpeed = 10.0;
    double currentSpeed = 20.0;
    double cmdOverride = bas.computeAccOverrideSpeed(3.0, frontVehicleSpeed, currentSpeed, 10.0);
    assertEquals(10.0, cmdOverride, 0.001);
  }

  @Before
  public void setUp() {
    bas = new BasicAccStrategy();
    bas.setDesiredTimeGap(1.0);
    bas.setMaxAccel(2.0);
    bas.setVehicleResponseDelay(0.0);
  }
}