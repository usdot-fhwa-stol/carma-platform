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

import java.util.List;
import java.util.ArrayList;

import org.junit.Before;
import org.junit.Test;
import org.mockito.Mock;
import org.mockito.MockitoAnnotations;

import cav_msgs.Maneuver;
import gov.dot.fhwa.saxton.carma.guidance.GuidanceCommands;
import gov.dot.fhwa.saxton.carma.guidance.trajectory.IManeuver.ManeuverType;

import static org.junit.Assert.*;
import static org.mockito.Mockito.*;

public class TrajectoryExecutorWorkerTest {

  @Before
  public void setup() {
    MockitoAnnotations.initMocks(this);
    tew = new TrajectoryExecutorWorker(guidanceCommands);
  }

  @Test
  public void testBasicTrajectory() {
    Trajectory t = mock(Trajectory.class);

    IManeuver m1 = mock(IManeuver.class);
    when(m1.getStartLocation()).thenReturn(0.0);
    when(m1.getEndLocation()).thenReturn(10.0);
    IManeuver m2 = mock(IManeuver.class);
    when(m2.getStartLocation()).thenReturn(10.0);
    when(m2.getEndLocation()).thenReturn(15.0);
    IManeuver m3 = mock(IManeuver.class);
    when(m3.getStartLocation()).thenReturn(15.0);
    when(m3.getEndLocation()).thenReturn(20.0);

    List<IManeuver> lonManeuvers = new ArrayList<IManeuver>();
    lonManeuvers.add(m1);
    lonManeuvers.add(m2);
    lonManeuvers.add(m3);

    List<IManeuver> latManeuvers = new ArrayList<IManeuver>();

    when(t.getLongitudinalManeuvers()).thenReturn(lonManeuvers);
    when(t.getLateralManeuvers()).thenReturn(latManeuvers);

    tew.runTrajectory(t);

    tew.updateDowntrackDistance(0.0);
    verify(m1).execute();
    tew.updateDowntrackDistance(10.0);
    verify(m2).execute();
    tew.updateDowntrackDistance(15.0);
    verify(m3).execute();
  }

  @Test
  public void testAbortTrajectory() {
    Trajectory t = mock(Trajectory.class);

    IManeuver m1 = mock(IManeuver.class);
    when(m1.getStartLocation()).thenReturn(0.0);
    when(m1.getEndLocation()).thenReturn(10.0);
    IManeuver m2 = mock(IManeuver.class);
    when(m2.getStartLocation()).thenReturn(10.0);
    when(m2.getEndLocation()).thenReturn(15.0);
    IManeuver m3 = mock(IManeuver.class);
    when(m3.getStartLocation()).thenReturn(15.0);
    when(m3.getEndLocation()).thenReturn(20.0);

    List<IManeuver> lonManeuvers = new ArrayList<IManeuver>();
    lonManeuvers.add(m1);
    lonManeuvers.add(m2);
    lonManeuvers.add(m3);

    List<IManeuver> latManeuvers = new ArrayList<IManeuver>();

    when(t.getLongitudinalManeuvers()).thenReturn(lonManeuvers);
    when(t.getLateralManeuvers()).thenReturn(latManeuvers);

    tew.runTrajectory(t);

    tew.updateDowntrackDistance(0.0);
    verify(m1).execute();
    tew.updateDowntrackDistance(10.0);
    verify(m2).execute();
    tew.abortTrajectory();

    assertNull(tew.getCurrentLateralManeuver());
    assertNull(tew.getCurrentLongitudinalManeuver());
  }

  @Test
  public void testGetCurrentManeuvers() {
    Trajectory t = mock(Trajectory.class);

    IManeuver m1 = mock(IManeuver.class);
    when(m1.getStartLocation()).thenReturn(0.0);
    when(m1.getEndLocation()).thenReturn(20.0);
    when(m1.getType()).thenReturn(ManeuverType.LATERAL);
    IManeuver m2 = mock(IManeuver.class);
    when(m2.getStartLocation()).thenReturn(0.0);
    when(m2.getEndLocation()).thenReturn(15.0);
    when(m2.getType()).thenReturn(ManeuverType.LONGITUDINAL);
    IManeuver m3 = mock(IManeuver.class);
    when(m3.getStartLocation()).thenReturn(15.0);
    when(m3.getEndLocation()).thenReturn(20.0);
    when(m3.getType()).thenReturn(ManeuverType.LONGITUDINAL);

    List<IManeuver> lonManeuvers = new ArrayList<IManeuver>();
    lonManeuvers.add(m1);
    lonManeuvers.add(m2);
    lonManeuvers.add(m3);

    List<IManeuver> latManeuvers = new ArrayList<IManeuver>();

    when(t.getLongitudinalManeuvers()).thenReturn(lonManeuvers);
    when(t.getLateralManeuvers()).thenReturn(latManeuvers);

    tew.runTrajectory(t);

    tew.updateDowntrackDistance(0.0);
    assertEquals(m1, tew.getCurrentLateralManeuver());
    assertEquals(m2, tew.getCurrentLongitudinalManeuver());
    tew.updateDowntrackDistance(15.0);
    assertEquals(m1, tew.getCurrentLateralManeuver());
    assertEquals(m3, tew.getCurrentLongitudinalManeuver());
  }

  @Test
  public void testGetNextManeuvers() {
    Trajectory t = mock(Trajectory.class);

    IManeuver m1 = mock(IManeuver.class);
    when(m1.getStartLocation()).thenReturn(0.0);
    when(m1.getEndLocation()).thenReturn(20.0);
    when(m1.getType()).thenReturn(ManeuverType.LATERAL);
    IManeuver m2 = mock(IManeuver.class);
    when(m2.getStartLocation()).thenReturn(00.0);
    when(m2.getEndLocation()).thenReturn(15.0);
    when(m2.getType()).thenReturn(ManeuverType.LONGITUDINAL);
    IManeuver m3 = mock(IManeuver.class);
    when(m3.getStartLocation()).thenReturn(15.0);
    when(m3.getEndLocation()).thenReturn(20.0);
    when(m3.getType()).thenReturn(ManeuverType.LONGITUDINAL);

    List<IManeuver> lonManeuvers = new ArrayList<IManeuver>();
    lonManeuvers.add(m1);
    lonManeuvers.add(m2);
    lonManeuvers.add(m3);

    List<IManeuver> latManeuvers = new ArrayList<IManeuver>();

    when(t.getLongitudinalManeuvers()).thenReturn(lonManeuvers);
    when(t.getLateralManeuvers()).thenReturn(latManeuvers);

    tew.runTrajectory(t);

    tew.updateDowntrackDistance(0.0);
    assertEquals(null, tew.getNextLateralManeuver());
    assertEquals(m3, tew.getNextLongitudinalManeuver());
    tew.updateDowntrackDistance(15.0);
    assertEquals(null, tew.getCurrentLateralManeuver());
    assertEquals(null, tew.getCurrentLongitudinalManeuver());
  }

  @Test
  public void testGetTrajectoryCompletionPct() {
    Trajectory t = mock(Trajectory.class);

    IManeuver m1 = mock(IManeuver.class);
    when(m1.getStartLocation()).thenReturn(0.0);
    when(m1.getEndLocation()).thenReturn(20.0);

    List<IManeuver> lonManeuvers = new ArrayList<IManeuver>();
    lonManeuvers.add(m1);

    when(t.getLongitudinalManeuvers()).thenReturn(lonManeuvers);

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
    assertEquals(1.0, tew.getTrajectoryCompletionPct(), 0.01);
  }

  @Test
  public void testTrajectoryProgressCallback() {
    Trajectory t = mock(Trajectory.class);

    IManeuver m1 = mock(IManeuver.class);
    when(m1.getStartLocation()).thenReturn(0.0);
    when(m1.getEndLocation()).thenReturn(20.0);

    List<IManeuver> lonManeuvers = new ArrayList<IManeuver>();
    lonManeuvers.add(m1);

    when(t.getLongitudinalManeuvers()).thenReturn(lonManeuvers);

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


  protected TrajectoryExecutorWorker tew;
  @Mock
  protected GuidanceCommands guidanceCommands;
}
