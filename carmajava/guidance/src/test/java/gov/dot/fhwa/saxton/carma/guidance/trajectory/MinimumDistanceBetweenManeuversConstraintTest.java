/*
 * TODO: Copyright (C) 2017 LEIDOS.
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

import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import static org.junit.Assert.*;

import static org.mockito.Mockito.*;

import gov.dot.fhwa.saxton.carma.guidance.trajectory.IManeuver.ManeuverType;

public class MinimumDistanceBetweenManeuversConstraintTest {
  @Before
  public void setup() {
    mdbmc = new MinimumDistanceBetweenManeuversConstraint(MIN_DIST);
  }

  @Test
  public void testSuccess() {
    IManeuver m1 = mock(IManeuver.class);
    when(m1.getStartLocation()).thenReturn(0.0);
    when(m1.getEndLocation()).thenReturn(1.0);
    when(m1.getType()).thenReturn(ManeuverType.LONGITUDINAL);

    IManeuver m2 = mock(IManeuver.class);
    when(m2.getStartLocation()).thenReturn(6.0);
    when(m2.getEndLocation()).thenReturn(7.0);
    when(m2.getType()).thenReturn(ManeuverType.LONGITUDINAL);

    IManeuver m3 = mock(IManeuver.class);
    when(m3.getStartLocation()).thenReturn(15.0);
    when(m3.getEndLocation()).thenReturn(20.0);
    when(m3.getType()).thenReturn(ManeuverType.LONGITUDINAL);

    mdbmc.visit(m1);
    mdbmc.visit(m2);
    mdbmc.visit(m3);

    assertTrue(mdbmc.getResult().getSuccess());
  }

  @Test
  public void testSuccessMixedTypes() {
    IManeuver m1 = mock(IManeuver.class);
    when(m1.getStartLocation()).thenReturn(0.0);
    when(m1.getEndLocation()).thenReturn(1.0);
    when(m1.getType()).thenReturn(ManeuverType.LONGITUDINAL);

    IManeuver m2 = mock(IManeuver.class);
    when(m2.getStartLocation()).thenReturn(2.0);
    when(m2.getEndLocation()).thenReturn(7.0);
    when(m2.getType()).thenReturn(ManeuverType.LATERAL);

    IManeuver m3 = mock(IManeuver.class);
    when(m3.getStartLocation()).thenReturn(15.0);
    when(m3.getEndLocation()).thenReturn(20.0);
    when(m3.getType()).thenReturn(ManeuverType.LONGITUDINAL);

    mdbmc.visit(m1);
    mdbmc.visit(m2);
    mdbmc.visit(m3);

    assertTrue(mdbmc.getResult().getSuccess());
  }

  @Test
  public void testRejection1() {
    IManeuver m1 = mock(IManeuver.class);
    when(m1.getStartLocation()).thenReturn(0.0);
    when(m1.getEndLocation()).thenReturn(1.0);
    when(m1.getType()).thenReturn(ManeuverType.LONGITUDINAL);

    IManeuver m2 = mock(IManeuver.class);
    when(m2.getStartLocation()).thenReturn(2.0);
    when(m2.getEndLocation()).thenReturn(7.0);
    when(m2.getType()).thenReturn(ManeuverType.LONGITUDINAL);

    mdbmc.visit(m1);
    mdbmc.visit(m2);

    assertFalse(mdbmc.getResult().getSuccess());
  }

  @Test
  public void testRejection2() {
    IManeuver m1 = mock(IManeuver.class);
    when(m1.getStartLocation()).thenReturn(0.0);
    when(m1.getEndLocation()).thenReturn(1.0);
    when(m1.getType()).thenReturn(ManeuverType.LONGITUDINAL);

    IManeuver m2 = mock(IManeuver.class);
    when(m2.getStartLocation()).thenReturn(6.0);
    when(m2.getEndLocation()).thenReturn(7.0);
    when(m2.getType()).thenReturn(ManeuverType.LONGITUDINAL);

    IManeuver m3 = mock(IManeuver.class);
    when(m3.getStartLocation()).thenReturn(7.0);
    when(m3.getEndLocation()).thenReturn(8.0);
    when(m3.getType()).thenReturn(ManeuverType.LONGITUDINAL);

    mdbmc.visit(m1);
    mdbmc.visit(m2);
    mdbmc.visit(m3);

    assertFalse(mdbmc.getResult().getSuccess());
  }

  protected MinimumDistanceBetweenManeuversConstraint mdbmc;
  protected final static double MIN_DIST = 5.0;
}
