/*
 * Copyright (C) 2018 LEIDOS. *
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
import org.mockito.Mock;
import org.mockito.MockitoAnnotations;

import gov.dot.fhwa.saxton.carma.guidance.GuidanceCommands;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.IComplexManeuver;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.ISimpleManeuver;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.LateralManeuver;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.LongitudinalManeuver;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.ManeuverType;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.IPublisher;
import gov.dot.fhwa.saxton.carma.guidance.util.ILogger;
import gov.dot.fhwa.saxton.carma.guidance.util.ILoggerFactory;
import gov.dot.fhwa.saxton.carma.guidance.util.LoggerManager;

import static org.junit.Assert.*;
import static org.mockito.Mockito.*;

public class TrajectoryExecutorWorkerTest {

  @Before
  public void setup() {
    ILoggerFactory mockFact = mock(ILoggerFactory.class);
    ILogger mockLogger = mock(ILogger.class);
    when(mockFact.createLoggerForClass(anyObject())).thenReturn(mockLogger);
    LoggerManager.setLoggerFactory(mockFact);
    
    MockitoAnnotations.initMocks(this);
    IPublisher<cav_msgs.ActiveManeuvers> pub = (IPublisher<cav_msgs.ActiveManeuvers>) mock(IPublisher.class);
    tew = new TrajectoryExecutorWorker(guidanceCommands, 10.0, pub);
  }

  private ISimpleManeuver newManeuver(double start, double end, ManeuverType type, boolean running) {
    if (type == ManeuverType.LONGITUDINAL) {
      ISimpleManeuver m1 = mock(LongitudinalManeuver.class);
      when(m1.getStartDistance()).thenReturn(start);
      when(m1.getEndDistance()).thenReturn(end);

      return m1;
    } else {
      ISimpleManeuver m1 = mock(LateralManeuver.class);
      when(m1.getStartDistance()).thenReturn(start);
      when(m1.getEndDistance()).thenReturn(end);

      return m1;
    }
  }

  @Test
  public void testBasicTrajectory() throws InterruptedException {
    Trajectory t = new Trajectory(0.0, 20.0);

    ISimpleManeuver m1 = newManeuver(0.0, 10.0, ManeuverType.LONGITUDINAL, false);
    ISimpleManeuver m2 = newManeuver(10.0, 15.0, ManeuverType.LONGITUDINAL, false);
    ISimpleManeuver m3 = newManeuver(15, 20.0, ManeuverType.LONGITUDINAL, false);

    ISimpleManeuver m1a = newManeuver(0.0, 10.0, ManeuverType.LATERAL, false);
    ISimpleManeuver m2a = newManeuver(10.0, 15.0, ManeuverType.LATERAL, false);
    ISimpleManeuver m3a = newManeuver(15, 20.0, ManeuverType.LATERAL, false);

    t.addManeuver(m1);
    t.addManeuver(m2);
    t.addManeuver(m3);
    t.addManeuver(m1a);
    t.addManeuver(m2a);
    t.addManeuver(m3a);

    tew.runTrajectory(t);

    tew.updateDowntrackDistance(0.0);
    tew.loop();
    verify(m1, atLeastOnce()).executeTimeStep();
    verify(m1a, atLeastOnce()).executeTimeStep();
    tew.updateDowntrackDistance(10.0);
    tew.loop();
    verify(m2, atLeastOnce()).executeTimeStep();
    verify(m2a, atLeastOnce()).executeTimeStep();
    tew.updateDowntrackDistance(15.0);
    tew.loop();
    verify(m3, atLeastOnce()).executeTimeStep();
    verify(m3a, atLeastOnce()).executeTimeStep();
  }

  @Test
  public void testAbortTrajectory() throws InterruptedException {
    Trajectory t = new Trajectory(0.0, 20.0);

    ISimpleManeuver m1 = newManeuver(0.0, 10.0, ManeuverType.LATERAL, false);
    ISimpleManeuver m2 = newManeuver(10.0, 15.0, ManeuverType.LATERAL, false);
    ISimpleManeuver m3 = newManeuver(15.0, 20.0, ManeuverType.LATERAL, false);
    ISimpleManeuver m4 = newManeuver(0.0, 20.0, ManeuverType.LONGITUDINAL, false);

    t.addManeuver(m1);
    t.addManeuver(m2);
    t.addManeuver(m3);
    t.addManeuver(m4);

    tew.runTrajectory(t);

    tew.updateDowntrackDistance(0.0);
    tew.loop();
    verify(m1, atLeastOnce()).executeTimeStep();
    tew.updateDowntrackDistance(10.0);
    tew.loop();
    verify(m2, atLeastOnce()).executeTimeStep();
    tew.abortTrajectory();

    assertNull(tew.getCurrentLateralManeuver());
    assertNull(tew.getCurrentLongitudinalManeuver());
  }

  @Test
  public void testGetCurrentManeuvers() {
    Trajectory t = new Trajectory(0.0, 20.0);

    ISimpleManeuver m1 = newManeuver(0.0, 20.0, ManeuverType.LATERAL, false);
    ISimpleManeuver m2 = newManeuver(0.0, 15.0, ManeuverType.LONGITUDINAL, false);
    ISimpleManeuver m3 = newManeuver(15.0, 20.0, ManeuverType.LONGITUDINAL, false);

    t.addManeuver(m1);
    t.addManeuver(m2);
    t.addManeuver(m3);

    tew.runTrajectory(t);

    tew.updateDowntrackDistance(0.0);
    tew.loop();
    assertEquals(m1, tew.getCurrentLateralManeuver());
    assertEquals(m2, tew.getCurrentLongitudinalManeuver());
    tew.updateDowntrackDistance(15.0);
    tew.loop();
    assertEquals(m1, tew.getCurrentLateralManeuver());
    assertEquals(m3, tew.getCurrentLongitudinalManeuver());
  }

  @Test
  public void testGetNextManeuvers() {
    Trajectory t = new Trajectory(0.0, 20.0);

    ISimpleManeuver m1 = newManeuver(0.0, 20.0, ManeuverType.LATERAL, false);
    ISimpleManeuver m2 = newManeuver(0.0, 15.0, ManeuverType.LONGITUDINAL, false);
    ISimpleManeuver m3 = newManeuver(15.0, 20.0, ManeuverType.LONGITUDINAL, false);

    t.addManeuver(m1);
    t.addManeuver(m2);
    t.addManeuver(m3);

    tew.runTrajectory(t);

    tew.updateDowntrackDistance(0.0);
    tew.loop();
    assertEquals(null, tew.getNextLateralManeuver());
    assertEquals(m3, tew.getNextLongitudinalManeuver());
    tew.updateDowntrackDistance(15.0);
    tew.loop();
    assertEquals(null, tew.getNextLateralManeuver());
    assertEquals(null, tew.getNextLongitudinalManeuver());
  }

  @Test
  public void testGetTrajectoryCompletionPct() {
    Trajectory t = new Trajectory(0.0, 20.0);

    ISimpleManeuver m1 = newManeuver(0.0, 20.0, ManeuverType.LATERAL, false);

    ISimpleManeuver m2 = newManeuver(0.0, 20.0, ManeuverType.LONGITUDINAL, false);

    t.addManeuver(m1);
    t.addManeuver(m2);

    tew.runTrajectory(t);

    tew.updateDowntrackDistance(0.0);
    assertEquals(0.0, tew.getTrajectoryCompletionPct(), 0.01);
    tew.updateDowntrackDistance(5.0);
    assertEquals(0.25, tew.getTrajectoryCompletionPct(), 0.01);
    tew.updateDowntrackDistance(10.0);
    assertEquals(0.50, tew.getTrajectoryCompletionPct(), 0.01);
    tew.updateDowntrackDistance(15.0);
    assertEquals(0.75, tew.getTrajectoryCompletionPct(), 0.01);
    tew.updateDowntrackDistance(20.0);
    double tcpct = tew.getTrajectoryCompletionPct();
    assertEquals(1.0, tew.getTrajectoryCompletionPct(), 0.01);
  }

  @Test
  public void testTrajectoryProgressCallback() {
    Trajectory t = new Trajectory(0.0, 20.0);

    ISimpleManeuver m1 = newManeuver(0.0, 20.0, ManeuverType.LONGITUDINAL, false);

    ISimpleManeuver m2 = newManeuver(0.0, 20.0, ManeuverType.LATERAL, false);

    t.addManeuver(m1);
    t.addManeuver(m2);

    OnTrajectoryProgressCallback cb1 = mock(OnTrajectoryProgressCallback.class);
    OnTrajectoryProgressCallback cb2 = mock(OnTrajectoryProgressCallback.class);
    OnTrajectoryProgressCallback cb3 = mock(OnTrajectoryProgressCallback.class);
    tew.registerOnTrajectoryProgressCallback(0.25, cb1);
    tew.registerOnTrajectoryProgressCallback(0.50, cb2);
    tew.registerOnTrajectoryProgressCallback(1.0, cb3);

    tew.runTrajectory(t);

    tew.updateDowntrackDistance(0.0);
    tew.updateDowntrackDistance(5.0);
    verify(cb1).onProgress(0.25);
    tew.updateDowntrackDistance(10.0);
    verify(cb2).onProgress(0.5);
    tew.updateDowntrackDistance(20.0);
    verify(cb3).onProgress(1.0);
  }

  @Test
  public void testComplexManeuverExecution1() throws InterruptedException {
    Trajectory t = new Trajectory(0.0, 30.0);

    IComplexManeuver m3 = mock(IComplexManeuver.class);
    when(m3.getStartDistance()).thenReturn(0.0);
    when(m3.getEndDistance()).thenReturn(30.0);

    t.setComplexManeuver(m3);

    tew.runTrajectory(t);

    tew.updateDowntrackDistance(0.0);
    tew.loop();
    verify(m3, atLeastOnce()).executeTimeStep();
    tew.updateDowntrackDistance(10.0);
    tew.loop();
    verify(m3, atLeastOnce()).executeTimeStep();
    tew.updateDowntrackDistance(20.0);
    tew.loop();
    verify(m3, atLeastOnce()).executeTimeStep();
    tew.loop();
    verify(m3, atLeastOnce()).executeTimeStep();
    tew.updateDowntrackDistance(30.0);
    tew.loop();

    assertEquals(null, tew.getCurrentComplexManeuver());
  }

  @Test
  public void testComplexManeuverExecution2() throws InterruptedException {
    Trajectory t = new Trajectory(0.0, 30.0);

    ISimpleManeuver m1 = newManeuver(0.0, 20.0, ManeuverType.LONGITUDINAL, false);

    ISimpleManeuver m2 = newManeuver(0.0, 20.0, ManeuverType.LATERAL, false);

    IComplexManeuver m3 = mock(IComplexManeuver.class);
    when(m3.getStartDistance()).thenReturn(20.0);
    when(m3.getEndDistance()).thenReturn(30.0);

    t.addManeuver(m1);
    t.addManeuver(m2);
    t.setComplexManeuver(m3);

    tew.runTrajectory(t);

    tew.updateDowntrackDistance(0.0);
    tew.updateDowntrackDistance(5.0);
    tew.updateDowntrackDistance(10.0);
    tew.updateDowntrackDistance(15.0);
    tew.updateDowntrackDistance(20.0);
    tew.loop();
    verify(m3, atLeastOnce()).executeTimeStep();
    tew.updateDowntrackDistance(25.0);
    tew.loop();
    verify(m3, atLeastOnce()).executeTimeStep();
    tew.updateDowntrackDistance(30.0);
    tew.loop();
    verify(m3, atLeastOnce()).executeTimeStep();
  }

  protected TrajectoryExecutorWorker tew;
  @Mock
  protected GuidanceCommands guidanceCommands;
}
