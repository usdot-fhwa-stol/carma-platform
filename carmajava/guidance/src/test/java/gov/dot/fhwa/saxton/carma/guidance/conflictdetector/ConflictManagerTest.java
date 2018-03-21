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

package gov.dot.fhwa.saxton.carma.guidance.conflictdetector;

import java.util.List;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedList;
import org.apache.commons.logging.Log;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import org.ros.message.MessageFactory;
import org.ros.message.Time;
import org.ros.node.NodeConfiguration;
import org.ros.rosjava_geometry.Transform;
import org.ros.rosjava_geometry.Vector3;
import org.ros.time.TimeProvider;

import static org.junit.Assert.*;
import static org.mockito.Mockito.*;

import gov.dot.fhwa.saxton.carma.geometry.GeodesicCartesianConverter;
import gov.dot.fhwa.saxton.carma.geometry.cartesian.Point;
import gov.dot.fhwa.saxton.carma.geometry.cartesian.Point3D;
import gov.dot.fhwa.saxton.carma.geometry.geodesic.Location;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.FutureLateralManeuver;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.IManeuver;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.ISimpleManeuver;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.LaneChange;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.LaneKeeping;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.LongitudinalManeuver;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.SlowDown;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.SpeedUp;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.SteadySpeed;
import gov.dot.fhwa.saxton.carma.guidance.plugins.IPlugin;
import gov.dot.fhwa.saxton.carma.guidance.trajectory.Trajectory;
import gov.dot.fhwa.saxton.carma.guidance.util.ILogger;
import gov.dot.fhwa.saxton.carma.guidance.util.ILoggerFactory;
import gov.dot.fhwa.saxton.carma.guidance.util.LoggerManager;
import gov.dot.fhwa.saxton.carma.guidance.util.trajectoryconverter.LongitudinalSimulationData;
import gov.dot.fhwa.saxton.carma.guidance.util.trajectoryconverter.RoutePointStamped;
import gov.dot.fhwa.saxton.carma.route.FileStrategy;
import gov.dot.fhwa.saxton.carma.route.Route;
import gov.dot.fhwa.saxton.carma.route.RouteSegment;

public class ConflictManagerTest {

  private IPlugin mockPlugin;
  private Route route;
  private cav_msgs.Route routeMsg;


  private Log log;
  private NodeConfiguration nodeConfiguration = NodeConfiguration.newPrivate();
  private MessageFactory messageFactory = nodeConfiguration.getTopicMessageFactory();
  private GeodesicCartesianConverter gcc = new GeodesicCartesianConverter();

  @Before
  public void setup() {
    ILoggerFactory mockFact = mock(ILoggerFactory.class);
    ILogger mockLogger = mock(ILogger.class);
    when(mockFact.createLoggerForClass(anyObject())).thenReturn(mockLogger);
    LoggerManager.setLoggerFactory(mockFact);
    mockPlugin = mock(IPlugin.class);
    log = mock(Log.class);
    route = (new FileStrategy("../route/src/test/resources/routes/colonial_farm_rd_outbound.yaml", log)).load();
    routeMsg = route.toMessage(messageFactory);
  }

  @Test
  public void testAddAndRemoveMobilityPaths() {
    double[] cellSize = {1,1,1};
    double downtrackMargin = 0.5;
    double crosstrackMargin = 0.5;
    double timeMargin = 0.5;
    MockTimeProvider timeProvider = new MockTimeProvider();
    timeProvider.setCurrentTime(0.0);
    ConflictManager cm = new ConflictManager(cellSize, downtrackMargin, crosstrackMargin, timeMargin, timeProvider);
    // Build path
    List<RoutePointStamped> path = new ArrayList<>();
    RoutePointStamped rp = new RoutePointStamped(0, 0, 0);
    rp.setSegDowntrack(0);
    rp.setSegment(routeMsg.getSegments().get(0));
    path.add(rp);
    rp = new RoutePointStamped(0.5, 0, 0.5);
    rp.setSegDowntrack(0.5);
    rp.setSegment(routeMsg.getSegments().get(0));
    path.add(rp);

    // Add path
    assertTrue(cm.addMobilityPath(path, "veh1"));
    assertFalse(cm.getConflicts(path).isEmpty());
    // Remove path
    assertTrue(cm.removeMobilityPath("veh1"));
    assertTrue(cm.getConflicts(path).isEmpty());
    // Add multiple paths
    assertTrue(cm.addMobilityPath(path, "veh1"));
    assertTrue(cm.addMobilityPath(path, "veh2"));
    assertFalse(cm.getConflicts(path).isEmpty());
    // Remove one path
    assertTrue(cm.removeMobilityPath("veh1"));
    assertFalse(cm.getConflicts(path).isEmpty());
    // Remove non-added path
    assertFalse(cm.removeMobilityPath("veh1"));
    assertFalse(cm.getConflicts(path).isEmpty());
    // Remove all paths
    assertTrue(cm.removeMobilityPath("veh2"));
    assertTrue(cm.getConflicts(path).isEmpty());
    // Add duplicate vehicles
    assertTrue(cm.addMobilityPath(path, "veh1"));
    assertTrue(cm.addMobilityPath(path, "veh1"));
    assertFalse(cm.getConflicts(path).isEmpty());
    assertTrue(cm.removeMobilityPath("veh1"));
    assertTrue(cm.getConflicts(path).isEmpty());

    // Test null path
    assertFalse(cm.addMobilityPath(null, "veh1"));
    // Test empty path
    assertFalse(cm.addMobilityPath(new LinkedList<>(), "veh1"));
    // Test null string
    assertFalse(cm.addMobilityPath(path, null));
    // Test remove null string
    assertFalse(cm.removeMobilityPath(null));
  }
  
  @Test
  public void testAddAndRemoveRequestedPaths() {
    double[] cellSize = {1,1,1};
    double downtrackMargin = 0.5;
    double crosstrackMargin = 0.5;
    double timeMargin = 0.5;
    MockTimeProvider timeProvider = new MockTimeProvider();
    timeProvider.setCurrentTime(0.0);
    ConflictManager cm = new ConflictManager(cellSize, downtrackMargin, crosstrackMargin, timeMargin, timeProvider);
    // Build path
    List<RoutePointStamped> path = new ArrayList<>();
    RoutePointStamped rp = new RoutePointStamped(0, 0, 0);
    rp.setSegDowntrack(0);
    rp.setSegment(routeMsg.getSegments().get(0));
    path.add(rp);
    rp = new RoutePointStamped(0.5, 0, 0.5);
    rp.setSegDowntrack(0.5);
    rp.setSegment(routeMsg.getSegments().get(0));
    path.add(rp);

    // Add path
    assertTrue(cm.addRequestedPath(path, "veh1"));
    assertFalse(cm.getConflicts(path).isEmpty());
    // Remove path
    assertTrue(cm.removeRequestedPath("veh1"));
    assertTrue(cm.getConflicts(path).isEmpty());
    // Add multiple paths
    assertTrue(cm.addRequestedPath(path, "veh1"));
    assertTrue(cm.addRequestedPath(path, "veh2"));
    assertFalse(cm.getConflicts(path).isEmpty());
    // Remove one path
    assertTrue(cm.removeRequestedPath("veh1"));
    assertFalse(cm.getConflicts(path).isEmpty());
    // Remove non-added path
    assertFalse(cm.removeRequestedPath("veh1"));
    assertFalse(cm.getConflicts(path).isEmpty());
    // Remove all paths
    assertTrue(cm.removeRequestedPath("veh2"));
    assertTrue(cm.getConflicts(path).isEmpty());
    // Add duplicate vehicles
    assertTrue(cm.addRequestedPath(path, "veh1"));
    assertTrue(cm.addRequestedPath(path, "veh1"));
    assertFalse(cm.getConflicts(path).isEmpty());
    assertTrue(cm.removeRequestedPath("veh1"));
    assertTrue(cm.getConflicts(path).isEmpty());

    // Test null path
    assertFalse(cm.addRequestedPath(null, "veh1"));
    // Test empty path
    assertFalse(cm.addRequestedPath(new LinkedList<>(), "veh1"));
    // Test null string
    assertFalse(cm.addRequestedPath(path, null));
    // Test remove null string
    assertFalse(cm.removeRequestedPath(null));
  }

  @Test
  public void testGetConflicts() {
    double[] cellSize = {1,1,1};
    double downtrackMargin = 0.5;
    double crosstrackMargin = 0.5;
    double timeMargin = 0.5;
    MockTimeProvider timeProvider = new MockTimeProvider();
    timeProvider.setCurrentTime(0.0);
    ConflictManager cm = new ConflictManager(cellSize, downtrackMargin, crosstrackMargin, timeMargin, timeProvider);
    // Build path
    List<RoutePointStamped> path = new ArrayList<>();
    RoutePointStamped rp = new RoutePointStamped(0, 0, 0);
    rp.setSegDowntrack(0);
    rp.setSegment(routeMsg.getSegments().get(0));
    path.add(rp);
    rp = new RoutePointStamped(0.5, 0, 0.5);
    rp.setSegDowntrack(0.5);
    rp.setSegment(routeMsg.getSegments().get(0));
    path.add(rp);

    // Add path
    assertTrue(cm.addMobilityPath(path, "veh1"));
    List<ConflictSpace> conflicts = cm.getConflicts(path);
    assertEquals(1, conflicts.size());
    assertEquals(0, conflicts.get(0).getStartDowntrack(), 0.0000001);
    assertEquals(0, conflicts.get(0).getStartTime(), 0.0000001);
    assertEquals(0.5, conflicts.get(0).getEndDowntrack(), 0.0000001);
    assertEquals(0.5, conflicts.get(0).getEndTime(), 0.0000001);
    // Remove path
    assertTrue(cm.removeMobilityPath("veh1"));
    assertTrue(cm.getConflicts(path).isEmpty());


    // Test null path
    assertFalse(cm.addMobilityPath(null, "veh1"));
    // Test empty path
    assertFalse(cm.addMobilityPath(new LinkedList<>(), "veh1"));
    // Test null string
    assertFalse(cm.addMobilityPath(path, null));
    // Test remove null string
    assertFalse(cm.removeMobilityPath(null));
  }
  
  @Test
  public void testGetConflictsBetweenPaths() {
    
  }
}
