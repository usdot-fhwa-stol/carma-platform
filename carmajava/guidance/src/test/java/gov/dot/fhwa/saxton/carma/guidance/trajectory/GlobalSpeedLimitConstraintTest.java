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

package gov.dot.fhwa.saxton.carma.guidance.trajectory;

import org.junit.Before;
import org.junit.Test;
import static org.junit.Assert.*;

import static org.mockito.Mockito.*;

import gov.dot.fhwa.saxton.carma.guidance.maneuvers.ISimpleManeuver;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.LongitudinalManeuver;

public class GlobalSpeedLimitConstraintTest {
  @Before
  public void setup() {
    gslc = new GlobalSpeedLimitConstraint(MAX_SPEED);
  }

  @Test
  public void testSuccess() {
    ISimpleManeuver m1 = mock(LongitudinalManeuver.class);
    when(m1.getStartSpeed()).thenReturn(0.0);
    when(m1.getTargetSpeed()).thenReturn(10.0);

    ISimpleManeuver m2 = mock(LongitudinalManeuver.class);
    when(m2.getStartSpeed()).thenReturn(10.0);
    when(m2.getTargetSpeed()).thenReturn(10.0);

    ISimpleManeuver m3 = mock(LongitudinalManeuver.class);
    when(m3.getStartSpeed()).thenReturn(10.0);
    when(m3.getTargetSpeed()).thenReturn(15.0);

    gslc.visit(m1);
    gslc.visit(m2);
    gslc.visit(m3);

    assertTrue(gslc.getResult().getSuccess());
  }

  @Test
  public void testSuccessMixedTypes() {
    ISimpleManeuver m1 = mock(LongitudinalManeuver.class);
    when(m1.getStartSpeed()).thenReturn(0.0);
    when(m1.getTargetSpeed()).thenReturn(1.0);

    ISimpleManeuver m2 = mock(ISimpleManeuver.class);

    ISimpleManeuver m3 = mock(LongitudinalManeuver.class);
    when(m3.getStartSpeed()).thenReturn(15.0);
    when(m3.getTargetSpeed()).thenReturn(19.0);

    gslc.visit(m1);
    gslc.visit(m2);
    gslc.visit(m3);

    assertTrue(gslc.getResult().getSuccess());
  }

  @Test
  public void testRejection() {
    ISimpleManeuver m1 = mock(LongitudinalManeuver.class);
    when(m1.getStartSpeed()).thenReturn(0.0);
    when(m1.getTargetSpeed()).thenReturn(10.0);

    ISimpleManeuver m2 = mock(LongitudinalManeuver.class);
    when(m2.getStartSpeed()).thenReturn(10.0);
    when(m2.getTargetSpeed()).thenReturn(25.0);

    gslc.visit(m1);
    gslc.visit(m2);

    assertFalse(gslc.getResult().getSuccess());
  }

  @Test
  public void testRejection2() {
    ISimpleManeuver m1 = mock(LongitudinalManeuver.class);
    when(m1.getStartSpeed()).thenReturn(21.0);
    when(m1.getTargetSpeed()).thenReturn(25.0);

    ISimpleManeuver m2 = mock(LongitudinalManeuver.class);
    when(m2.getStartSpeed()).thenReturn(25.0);
    when(m2.getTargetSpeed()).thenReturn(10.0);

    ISimpleManeuver m3 = mock(LongitudinalManeuver.class);
    when(m3.getStartSpeed()).thenReturn(10.0);
    when(m3.getTargetSpeed()).thenReturn(30.0);

    gslc.visit(m1);
    gslc.visit(m2);
    gslc.visit(m3);

    assertFalse(gslc.getResult().getSuccess());
  }

  protected GlobalSpeedLimitConstraint gslc;
  protected final static double MAX_SPEED = 20.0;
}
