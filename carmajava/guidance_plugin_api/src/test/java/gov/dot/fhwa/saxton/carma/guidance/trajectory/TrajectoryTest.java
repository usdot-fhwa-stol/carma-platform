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

package gov.dot.fhwa.saxton.carma.guidance.trajectory;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import java.util.List;

import static org.junit.Assert.*;
import static org.mockito.Mockito.*;

import gov.dot.fhwa.saxton.carma.guidance.maneuvers.IComplexManeuver;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.IManeuver;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.ISimpleManeuver;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.LateralManeuver;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.LongitudinalManeuver;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.ManeuverType;
import gov.dot.fhwa.saxton.carma.guidance.util.ILogger;
import gov.dot.fhwa.saxton.carma.guidance.util.ILoggerFactory;
import gov.dot.fhwa.saxton.carma.guidance.util.LoggerManager;

public class TrajectoryTest {
  @Before
  public void setup() {
    ILoggerFactory mockFact = mock(ILoggerFactory.class);
    ILogger mockLogger = mock(ILogger.class);
    when(mockFact.createLoggerForClass(anyObject())).thenReturn(mockLogger);
    LoggerManager.setLoggerFactory(mockFact);
    traj = new Trajectory(0, 20);
  }

  @After
  public void cleanup() {
    traj = null;
  }

  private LongitudinalManeuver createLongitudinalManeuver(double start, double end) {
    LongitudinalManeuver mvr = mock(LongitudinalManeuver.class);
    when(mvr.getStartDistance()).thenReturn(start);
    when(mvr.getEndDistance()).thenReturn(end);

    return mvr;
  }

  private LateralManeuver createLateralManeuver(double start, double end) {
    LateralManeuver mvr = mock(LateralManeuver.class);
    when(mvr.getStartDistance()).thenReturn(start);
    when(mvr.getEndDistance()).thenReturn(end);

    return mvr;
  }

  @Test
  public void testAddLateralManeuver() {
    assertTrue(traj.addManeuver(createLateralManeuver(0, 0)));
  }

  @Test
  public void testAddLateralManeuverRejection2() {
    assertFalse(traj.addManeuver(createLateralManeuver(0, 21)));
  }

  @Test
  public void testAddLateralManeuverRejection3() {
    assertFalse(traj.addManeuver(createLateralManeuver(21, 25)));
  }

  @Test
  public void testAddLongitudinalManeuver() {
    assertTrue(traj.addManeuver(createLongitudinalManeuver(0, 0)));
  }

  @Test
  public void testAddLongitudinalManeuverRejection2() {
    assertFalse(traj.addManeuver(createLongitudinalManeuver(0, 21)));
  }

  @Test
  public void testAddLongitudinalManeuverRejection3() {
    assertFalse(traj.addManeuver(createLongitudinalManeuver(21, 25)));
  }

  @Test
  public void testGetManeuversAt1() {
    ISimpleManeuver m = createLateralManeuver(0, 1);
    traj.addManeuver(m);

    List<IManeuver> maneuvers = traj.getManeuversAt(0.0);
    assertEquals(1, maneuvers.size());
    assertEquals(m, maneuvers.get(0));
  }

  @Test
  public void testGetManeuversAt2() {
    ISimpleManeuver m1 = createLateralManeuver(0, 1);
    ISimpleManeuver m2 = createLongitudinalManeuver(0, 1);
    traj.addManeuver(m1);
    traj.addManeuver(m2);

    List<IManeuver> maneuvers = traj.getManeuversAt(0.0);

    assertEquals(2, maneuvers.size());
    assertTrue(maneuvers.contains(m1));
    assertTrue(maneuvers.contains(m2));
  }

  @Test
  public void testGetManeuversAt3() {
    ISimpleManeuver m1 = createLateralManeuver(0, 10);
    ISimpleManeuver m2 = createLongitudinalManeuver(5, 15);
    traj.addManeuver(m1);
    traj.addManeuver(m2);

    List<IManeuver> maneuvers = traj.getManeuversAt(7.5);

    assertEquals(2, maneuvers.size());
    assertTrue(maneuvers.contains(m1));
    assertTrue(maneuvers.contains(m2));
  }

  @Test
  public void testFindEarliestWindowOfSize1() {
    traj.addManeuver(createLongitudinalManeuver(0, 5));
    traj.addManeuver(createLongitudinalManeuver(7, 10));

    double loc = traj.findEarliestLongitudinalWindowOfSize(2.0);

    assertEquals(5.0, loc, 0.01);
  }

  @Test
  public void testFindEarliestWindowOfSize2() {
    traj.addManeuver(createLongitudinalManeuver(0, 5));
    traj.addManeuver(createLongitudinalManeuver(7, 10));
    traj.addManeuver(createLongitudinalManeuver(15, 20));

    double loc = traj.findEarliestLongitudinalWindowOfSize(5.0);

    assertEquals(10.0, loc, 0.01);
  }

  @Test
  public void testFindEarliestWindowOfSize3() {
    traj.addManeuver(createLongitudinalManeuver(0, 5));
    traj.addManeuver(createLongitudinalManeuver(7, 10));
    traj.addManeuver(createLongitudinalManeuver(15, 20));

    double loc = traj.findEarliestLongitudinalWindowOfSize(2.0);

    assertEquals(5.0, loc, 0.01);
  }

  @Test
  public void testFindEarliestWindowOfSize4() {
    traj.addManeuver(createLongitudinalManeuver(0, 5));
    traj.addManeuver(createLongitudinalManeuver(5, 15));
    traj.addManeuver(createLongitudinalManeuver(15, 18));

    double loc = traj.findEarliestLongitudinalWindowOfSize(2.0);

    assertEquals(18.0, loc, 0.01);
  }

  @Test
  public void testFindEarliestWindowOfSize5() {
    traj.addManeuver(createLateralManeuver(0, 5));
    traj.addManeuver(createLateralManeuver(5, 15));
    traj.addManeuver(createLateralManeuver(15, 18));

    double loc = traj.findEarliestLateralWindowOfSize(2.0);

    assertEquals(18.0, loc, 0.01);
  }

  @Test
  public void testFindEarliestWindowOfSizeFail() {
    traj.addManeuver(createLongitudinalManeuver(0, 5));
    traj.addManeuver(createLongitudinalManeuver(7, 10));

    double loc = traj.findLatestLongitudinalWindowOfSize(11.0);

    assertEquals(-1.0, loc, 0.01);
  }

  @Test
  public void testFindLatestWindowOfSize1() {
    traj.addManeuver(createLongitudinalManeuver(0, 5));
    traj.addManeuver(createLongitudinalManeuver(7, 10));

    double loc = traj.findLatestLongitudinalWindowOfSize(2.0);

    assertEquals(10.0, loc, 0.01);
  }

  @Test
  public void testFindLatestWindowOfSize2() {
    traj.addManeuver(createLongitudinalManeuver(0, 5));
    traj.addManeuver(createLongitudinalManeuver(7, 10));
    traj.addManeuver(createLongitudinalManeuver(15, 20));

    double loc = traj.findLatestLongitudinalWindowOfSize(5.0);

    assertEquals(10.0, loc, 0.01);
  }

  @Test
  public void testFindLatestWindowOfSize3() {
    traj.addManeuver(createLongitudinalManeuver(0, 5));
    traj.addManeuver(createLongitudinalManeuver(7, 10));
    traj.addManeuver(createLongitudinalManeuver(15, 20));

    double loc = traj.findLatestLongitudinalWindowOfSize(2.0);

    assertEquals(10.0, loc, 0.01);
  }

  @Test
  public void testFindLatestWindowOfSizeFail() {
    traj.addManeuver(createLongitudinalManeuver(0, 5));
    traj.addManeuver(createLongitudinalManeuver(7, 10));

    double loc = traj.findLatestLongitudinalWindowOfSize(11.0);

    assertEquals(-1.0, loc, 0.01);
  }

  @Test
  public void testGetNextLateralManeuverAfterEmpty() {
    IManeuver m = traj.getNextManeuverAfter(0.0, ManeuverType.LATERAL);
    assertEquals(null, m);

    traj.addManeuver(createLongitudinalManeuver( 1.0, 2.0));
    m = traj.getNextManeuverAfter(0.0, ManeuverType.LATERAL);
    assertEquals(null, m);
  }

  @Test
  public void testGetNextLateralManeuverAfter1() {
    ISimpleManeuver m = createLateralManeuver(1.0, 2.0);
    traj.addManeuver(m);
    IManeuver m1 = traj.getNextManeuverAfter(0.0, ManeuverType.LATERAL);
    assertEquals(m, m1);
  }

  @Test
  public void testGetNextLateralManeuverAfter2() {
    traj.addManeuver(createLateralManeuver(0.0, 0.0));

    ISimpleManeuver m = createLateralManeuver(1.0, 2.0);
    traj.addManeuver(m);
    IManeuver m1 = traj.getNextManeuverAfter(0.0, ManeuverType.LATERAL);
    assertEquals(m, m1);
  }

  @Test
  public void testGetManeuvers1() {
    traj.addManeuver(createLongitudinalManeuver(1.14, 11.4));

    List<IManeuver> mvrs = traj.getManeuvers();
    IManeuver m = mvrs.get(0);

    assertEquals(1, mvrs.size());
    assertEquals(1.14, mvrs.get(0).getStartDistance(), 0.001);
    assertEquals(11.4, mvrs.get(0).getEndDistance(), 0.001);
  }

  @Test
  public void testGetManeuvers2() {
    ISimpleManeuver inputM1 = createLateralManeuver(1.14, 11.4);
    ISimpleManeuver inputM2 = createLateralManeuver(11.4, 20.0);
    traj.addManeuver(inputM1);
    traj.addManeuver(inputM2);

    List<IManeuver> mvrs = traj.getManeuvers();
    assertEquals(2, mvrs.size());
    IManeuver m1 = mvrs.get(0);
    IManeuver m2 = mvrs.get(1);

    assertEquals(inputM1, m1);
    assertEquals(inputM2, m2);
  }

  @Test
  public void testGetNextLateralManeuverAfterFail() {
    traj.addManeuver(createLateralManeuver( 0.0, 0.0));

    IManeuver m = traj.getNextManeuverAfter(0.0, ManeuverType.LATERAL);
    assertEquals(null, m);
  }

  @Test
  public void testGetNextLongitudinalManeuverAfterEmpty() {
    IManeuver m = traj.getNextManeuverAfter(0.0, ManeuverType.LONGITUDINAL);
    assertEquals(null, m);

    traj.addManeuver(createLateralManeuver( 1.0, 2.0));
    m = traj.getNextManeuverAfter(0.0, ManeuverType.LONGITUDINAL);
    assertEquals(null, m);
  }

  @Test
  public void testGetNextLongitudinalManeuverAfter1() {
    ISimpleManeuver m = createLongitudinalManeuver( 1.0, 2.0);
    traj.addManeuver(m);
    IManeuver m1 = traj.getNextManeuverAfter(0.0, ManeuverType.LONGITUDINAL);
    assertEquals(m, m1);
  }

  @Test
  public void testGetNextLongitudinalManeuverAfter2() {
    traj.addManeuver(createLongitudinalManeuver( 0.0, 0.0));

    ISimpleManeuver m = createLongitudinalManeuver( 1.0, 2.0);
    traj.addManeuver(m);
    IManeuver m1 = traj.getNextManeuverAfter(0.0, ManeuverType.LONGITUDINAL);
    assertEquals(m, m1);
  }

  @Test
  public void testGetNextLongitudinalManeuverAfterFail() {
    traj.addManeuver(createLongitudinalManeuver( 0.0, 0.0));

    IManeuver m = traj.getNextManeuverAfter(0.0, ManeuverType.LONGITUDINAL);
    assertEquals(null, m);
  }

  @Test
  public void testSetComplexManeuverSuccess() {
    IComplexManeuver complexManeuver = mock(IComplexManeuver.class);
    when(complexManeuver.getStartDistance()).thenReturn(0.0);
    when(complexManeuver.getEndDistance()).thenReturn(20.0);
    assertTrue(traj.setComplexManeuver(complexManeuver));
  }

  @Test
  public void testSetComplexManeuverLengthAdjustment() {
    IComplexManeuver complexManeuver = mock(IComplexManeuver.class);
    when(complexManeuver.getStartDistance()).thenReturn(0.0);
    when(complexManeuver.getEndDistance()).thenReturn(10.0);
    traj.setComplexManeuver(complexManeuver);
    assertEquals(10.0, traj.getEndLocation(), 0.001);
  }

  @Test
  public void testSetComplexManeuverFailure() {
    IComplexManeuver complexManeuver = mock(IComplexManeuver.class);
    when(complexManeuver.getStartDistance()).thenReturn(10.0);
    when(complexManeuver.getEndDistance()).thenReturn(30.0);
    assertFalse(traj.setComplexManeuver(complexManeuver));
    assertEquals(20.0, traj.getEndLocation(), 0.001);
  }

  protected Trajectory traj;
}