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

package gov.dot.fhwa.saxton.carma.guidance.trajectory;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import static org.junit.Assert.*;

import static org.mockito.Mockito.*;

import gov.dot.fhwa.saxton.carma.guidance.maneuvers.IManeuver;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.LongitudinalManeuver;
import gov.dot.fhwa.saxton.carma.guidance.util.ILogger;
import gov.dot.fhwa.saxton.carma.guidance.util.ILoggerFactory;
import gov.dot.fhwa.saxton.carma.guidance.util.LoggerManager;


public class OverlappingManeuversConstraintTest {

  @Before
  public void setup() {
    ILoggerFactory mockFact = mock(ILoggerFactory.class);
    ILogger mockLogger = mock(ILogger.class);
    when(mockFact.createLoggerForClass(anyObject())).thenReturn(mockLogger);
    LoggerManager.setLoggerFactory(mockFact);
    omc = new OverlappingManeuversConstraint();
  }

  @Test
  public void testSuccess() {
    IManeuver m1 = mock(LongitudinalManeuver.class);
    when(m1.getStartDistance()).thenReturn(0.0);
    when(m1.getEndDistance()).thenReturn(1.0);

    IManeuver m2 = mock(LongitudinalManeuver.class);
    when(m2.getStartDistance()).thenReturn(1.0);
    when(m2.getEndDistance()).thenReturn(2.0);

    IManeuver m3 = mock(LongitudinalManeuver.class);
    when(m3.getStartDistance()).thenReturn(5.0);
    when(m3.getEndDistance()).thenReturn(10.0);

    omc.visit(m1);
    omc.visit(m2);
    omc.visit(m3);

    assertTrue(omc.getResult().getSuccess());
  }

  @Test
  public void testMultiDomainSuccess() {
    IManeuver m1 = mock(LongitudinalManeuver.class);
    when(m1.getStartDistance()).thenReturn(0.0);
    when(m1.getEndDistance()).thenReturn(1.0);

    IManeuver m2 = mock(IManeuver.class);
    when(m2.getStartDistance()).thenReturn(0.0);
    when(m2.getEndDistance()).thenReturn(2.0);

    IManeuver m3 = mock(LongitudinalManeuver.class);
    when(m3.getStartDistance()).thenReturn(5.0);
    when(m3.getEndDistance()).thenReturn(10.0);

    omc.visit(m1);
    omc.visit(m2);
    omc.visit(m3);

    assertTrue(omc.getResult().getSuccess());
  }

  @Test
  public void testSingleManeuverRejection() {
    IManeuver m1 = mock(LongitudinalManeuver.class);
    when(m1.getStartDistance()).thenReturn(0.0);
    when(m1.getEndDistance()).thenReturn(1.0);

    IManeuver m2 = mock(LongitudinalManeuver.class);
    when(m2.getStartDistance()).thenReturn(0.0);
    when(m2.getEndDistance()).thenReturn(2.0);

    IManeuver m3 = mock(LongitudinalManeuver.class);
    when(m3.getStartDistance()).thenReturn(5.0);
    when(m3.getEndDistance()).thenReturn(10.0);

    omc.visit(m1);
    omc.visit(m2);
    omc.visit(m3);

    assertFalse(omc.getResult().getSuccess());
  }

  @Test
  public void testMultipleManeuverRejection() {
    IManeuver m1 = mock(LongitudinalManeuver.class);
    when(m1.getStartDistance()).thenReturn(0.0);
    when(m1.getEndDistance()).thenReturn(1.0);

    IManeuver m2 = mock(LongitudinalManeuver.class);
    when(m2.getStartDistance()).thenReturn(0.0);
    when(m2.getEndDistance()).thenReturn(2.0);

    IManeuver m3 = mock(LongitudinalManeuver.class);
    when(m3.getStartDistance()).thenReturn(0.0);
    when(m3.getEndDistance()).thenReturn(10.0);

    omc.visit(m1);
    omc.visit(m2);
    omc.visit(m3);

    assertFalse(omc.getResult().getSuccess());
  }

  private OverlappingManeuversConstraint omc;
}
