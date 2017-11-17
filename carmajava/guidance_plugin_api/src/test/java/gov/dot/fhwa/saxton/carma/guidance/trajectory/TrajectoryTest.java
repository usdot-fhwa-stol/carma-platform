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

import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import java.util.List;

import static org.junit.Assert.*;
import static org.mockito.Mockito.*;

import gov.dot.fhwa.saxton.carma.guidance.maneuvers.IManeuver;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.IManeuverInputs;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.ISimpleManeuver;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.LongitudinalManeuver;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.ManeuverType;
import gov.dot.fhwa.saxton.carma.guidance.util.ILogger;
import gov.dot.fhwa.saxton.carma.guidance.util.ILoggerFactory;
import gov.dot.fhwa.saxton.carma.guidance.util.LoggerManager;
import gov.dot.fhwa.saxton.carma.guidance.IGuidanceCommands;

class LongitudinalTestManeuver extends LongitudinalManeuver {
  LongitudinalTestManeuver(ManeuverType type, double start, double end) {
    this.type = type;
    this.start = start;
    this.end = end;
  }

  public ManeuverType getType() {
    return type;
  }

  @Override
  public double getStartDistance() {
    return start;
  }

  @Override
  public double getEndDistance() {
    return end;
  }

  @Override
  public void setSpeeds(double startSpeed, double targetSpeed) throws UnsupportedOperationException {

  }

  @Override
  public double getStartSpeed() throws UnsupportedOperationException {
    return 0;
  }

  @Override
  public double getTargetSpeed() throws UnsupportedOperationException {
    return 0;
  }

  @Override
  public void setTargetLane(int targetLane) throws UnsupportedOperationException {

  }

  private double end;
  private double start;
  private ManeuverType type;
@Override
public double generateSpeedCommand() {
	return 0;
}

}

// Bit of a hack to deal with how maneuvers are differentiated now
class LateralTestManeuver implements ISimpleManeuver {
  LateralTestManeuver(ManeuverType type, double start, double end) {
    this.type = type;
    this.start = start;
    this.end = end;
  }

  public ManeuverType getType() {
    return type;
  }

  @Override
  public double getStartDistance() {
    return start;
  }

  @Override
  public double getEndDistance() {
    return end;
  }

  @Override
  public void plan(IManeuverInputs inputs, IGuidanceCommands commands, double startDist) throws IllegalStateException {

  }

  @Override
  public void planToTargetDistance(IManeuverInputs inputs, IGuidanceCommands commands, double startDist, double endDist) throws IllegalStateException {

  }

  @Override
  public boolean executeTimeStep() throws IllegalStateException {
    return false;
  }

  @Override
  public void setSpeeds(double startSpeed, double targetSpeed) throws UnsupportedOperationException {

  }

  @Override
  public double getStartSpeed() throws UnsupportedOperationException {
    return 0;
  }

  @Override
  public double getTargetSpeed() throws UnsupportedOperationException {
    return 0;
  }

  @Override
  public void setTargetLane(int targetLane) throws UnsupportedOperationException {

  }

  private double end;
  private double start;
  private ManeuverType type;

}

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

  @Test
  public void testAddLateralManeuver() {
    assertTrue(traj.addManeuver(new LateralTestManeuver(ManeuverType.LATERAL, 0, 0)));
  }

  @Test
  public void testAddLateralManeuverRejection2() {
    assertFalse(traj.addManeuver(new LateralTestManeuver(ManeuverType.LATERAL, 0, 21)));
  }

  @Test
  public void testAddLateralManeuverRejection3() {
    assertFalse(traj.addManeuver(new LateralTestManeuver(ManeuverType.LATERAL, 21, 25)));
  }

  @Test
  public void testAddLongitudinalManeuver() {
    assertTrue(traj.addManeuver(new LongitudinalTestManeuver(ManeuverType.LONGITUDINAL, 0, 0)));
  }

  @Test
  public void testAddLongitudinalManeuverRejection2() {
    assertFalse(traj.addManeuver(new LongitudinalTestManeuver(ManeuverType.LONGITUDINAL, 0, 21)));
  }

  @Test
  public void testAddLongitudinalManeuverRejection3() {
    assertFalse(traj.addManeuver(new LongitudinalTestManeuver(ManeuverType.LONGITUDINAL, 21, 25)));
  }

  @Test
  public void testGetManeuversAt1() {
    ISimpleManeuver m = new LateralTestManeuver(ManeuverType.LATERAL, 0, 1);
    traj.addManeuver(m);

    List<IManeuver> maneuvers = traj.getManeuversAt(0.0);
    assertEquals(1, maneuvers.size());
    assertEquals(m, maneuvers.get(0));
  }

  @Test
  public void testGetManeuversAt2() {
    ISimpleManeuver m1 = new LateralTestManeuver(ManeuverType.LATERAL, 0, 1);
    ISimpleManeuver m2 = new LongitudinalTestManeuver(ManeuverType.LONGITUDINAL, 0, 1);
    traj.addManeuver(m1);
    traj.addManeuver(m2);

    List<IManeuver> maneuvers = traj.getManeuversAt(0.0);

    assertEquals(2, maneuvers.size());
    assertTrue(maneuvers.contains(m1));
    assertTrue(maneuvers.contains(m2));
  }

  @Test
  public void testGetManeuversAt3() {
    ISimpleManeuver m1 = new LateralTestManeuver(ManeuverType.LATERAL, 0, 10);
    ISimpleManeuver m2 = new LongitudinalTestManeuver(ManeuverType.LONGITUDINAL, 5, 15);
    traj.addManeuver(m1);
    traj.addManeuver(m2);

    List<IManeuver> maneuvers = traj.getManeuversAt(7.5);

    assertEquals(2, maneuvers.size());
    assertTrue(maneuvers.contains(m1));
    assertTrue(maneuvers.contains(m2));
  }

  @Test
  public void testFindEarliestWindowOfSize1() {
    traj.addManeuver(new LongitudinalTestManeuver(ManeuverType.LONGITUDINAL, 0, 5));
    traj.addManeuver(new LongitudinalTestManeuver(ManeuverType.LONGITUDINAL, 7, 10));

    double loc = traj.findEarliestWindowOfSize(2.0);

    assertEquals(5.0, loc, 0.01);
  }

  @Test
  public void testFindEarliestWindowOfSize2() {
    traj.addManeuver(new LongitudinalTestManeuver(ManeuverType.LONGITUDINAL, 0, 5));
    traj.addManeuver(new LongitudinalTestManeuver(ManeuverType.LONGITUDINAL, 7, 10));
    traj.addManeuver(new LongitudinalTestManeuver(ManeuverType.LONGITUDINAL, 15, 20));

    double loc = traj.findEarliestWindowOfSize(5.0);

    assertEquals(10.0, loc, 0.01);
  }

  @Test
  public void testFindEarliestWindowOfSize3() {
    traj.addManeuver(new LongitudinalTestManeuver(ManeuverType.LONGITUDINAL, 0, 5));
    traj.addManeuver(new LongitudinalTestManeuver(ManeuverType.LONGITUDINAL, 7, 10));
    traj.addManeuver(new LongitudinalTestManeuver(ManeuverType.LONGITUDINAL, 15, 20));

    double loc = traj.findEarliestWindowOfSize(2.0);

    assertEquals(5.0, loc, 0.01);
  }

  @Test
  public void testFindEarliestWindowOfSizeFail() {
    traj.addManeuver(new LongitudinalTestManeuver(ManeuverType.LONGITUDINAL, 0, 5));
    traj.addManeuver(new LongitudinalTestManeuver(ManeuverType.LONGITUDINAL, 7, 10));

    double loc = traj.findLatestWindowOfSize(3.0);

    assertEquals(-1.0, loc, 0.01);
  }

  @Test
  public void testFindLatestWindowOfSize1() {
    traj.addManeuver(new LongitudinalTestManeuver(ManeuverType.LONGITUDINAL, 0, 5));
    traj.addManeuver(new LongitudinalTestManeuver(ManeuverType.LONGITUDINAL, 7, 10));

    double loc = traj.findLatestWindowOfSize(2.0);

    assertEquals(5.0, loc, 0.01);
  }

  @Test
  public void testFindLatestWindowOfSize2() {
    traj.addManeuver(new LongitudinalTestManeuver(ManeuverType.LONGITUDINAL, 0, 5));
    traj.addManeuver(new LongitudinalTestManeuver(ManeuverType.LONGITUDINAL, 7, 10));
    traj.addManeuver(new LongitudinalTestManeuver(ManeuverType.LONGITUDINAL, 15, 20));

    double loc = traj.findLatestWindowOfSize(5.0);

    assertEquals(10.0, loc, 0.01);
  }

  @Test
  public void testFindLatestWindowOfSize3() {
    traj.addManeuver(new LongitudinalTestManeuver(ManeuverType.LONGITUDINAL, 0, 5));
    traj.addManeuver(new LongitudinalTestManeuver(ManeuverType.LONGITUDINAL, 7, 10));
    traj.addManeuver(new LongitudinalTestManeuver(ManeuverType.LONGITUDINAL, 15, 20));

    double loc = traj.findLatestWindowOfSize(2.0);

    assertEquals(10.0, loc, 0.01);
  }

  @Test
  public void testFindLatestWindowOfSizeFail() {
    traj.addManeuver(new LongitudinalTestManeuver(ManeuverType.LONGITUDINAL, 0, 5));
    traj.addManeuver(new LongitudinalTestManeuver(ManeuverType.LONGITUDINAL, 7, 10));

    double loc = traj.findLatestWindowOfSize(3.0);

    assertEquals(-1.0, loc, 0.01);
  }

  @Test
  public void testGetNextLateralManeuverAfterEmpty() {
    IManeuver m = traj.getNextManeuverAfter(0.0, ManeuverType.LATERAL);
    assertEquals(null, m);

    traj.addManeuver(new LongitudinalTestManeuver(ManeuverType.LONGITUDINAL, 1.0, 2.0));
    m = traj.getNextManeuverAfter(0.0, ManeuverType.LATERAL);
    assertEquals(null, m);
  }

  @Test
  public void testGetNextLateralManeuverAfter1() {
    ISimpleManeuver m = new LateralTestManeuver(ManeuverType.LATERAL, 1.0, 2.0);
    traj.addManeuver(m);
    IManeuver m1 = traj.getNextManeuverAfter(0.0, ManeuverType.LATERAL);
    assertEquals(m, m1);
  }

  @Test
  public void testGetNextLateralManeuverAfter2() {
    traj.addManeuver(new LateralTestManeuver(ManeuverType.LATERAL, 0.0, 0.0));

    ISimpleManeuver m = new LateralTestManeuver(ManeuverType.LATERAL, 1.0, 2.0);
    traj.addManeuver(m);
    IManeuver m1 = traj.getNextManeuverAfter(0.0, ManeuverType.LATERAL);
    assertEquals(m, m1);
  }

  @Test
  public void testGetNextLateralManeuverAfterFail() {
    traj.addManeuver(new LateralTestManeuver(ManeuverType.LATERAL, 0.0, 0.0));

    IManeuver m = traj.getNextManeuverAfter(0.0, ManeuverType.LATERAL);
    assertEquals(null, m);
  }

  @Test
  public void testGetNextLongitudinalManeuverAfterEmpty() {
    IManeuver m = traj.getNextManeuverAfter(0.0, ManeuverType.LONGITUDINAL);
    assertEquals(null, m);

    traj.addManeuver(new LateralTestManeuver(ManeuverType.LATERAL, 1.0, 2.0));
    m = traj.getNextManeuverAfter(0.0, ManeuverType.LONGITUDINAL);
    assertEquals(null, m);
  }

  @Test
  public void testGetNextLongitudinalManeuverAfter1() {
    ISimpleManeuver m = new LongitudinalTestManeuver(ManeuverType.LONGITUDINAL, 1.0, 2.0);
    traj.addManeuver(m);
    IManeuver m1 = traj.getNextManeuverAfter(0.0, ManeuverType.LONGITUDINAL);
    assertEquals(m, m1);
  }

  @Test
  public void testGetNextLongitudinalManeuverAfter2() {
    traj.addManeuver(new LongitudinalTestManeuver(ManeuverType.LONGITUDINAL, 0.0, 0.0));

    ISimpleManeuver m = new LongitudinalTestManeuver(ManeuverType.LONGITUDINAL, 1.0, 2.0);
    traj.addManeuver(m);
    IManeuver m1 = traj.getNextManeuverAfter(0.0, ManeuverType.LONGITUDINAL);
    assertEquals(m, m1);
  }

  @Test
  public void testGetNextLongitudinalManeuverAfterFail() {
    traj.addManeuver(new LongitudinalTestManeuver(ManeuverType.LONGITUDINAL, 0.0, 0.0));

    IManeuver m = traj.getNextManeuverAfter(0.0, ManeuverType.LONGITUDINAL);
    assertEquals(null, m);
  }

  protected Trajectory traj;
}