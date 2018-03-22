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
import gov.dot.fhwa.saxton.carma.guidance.conflictdetector.ConflictSpace;
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
    assertTrue(cm.addRequestedPath(path, "plan1", "veh1"));
    assertFalse(cm.getConflicts(path).isEmpty());
    // Remove path
    assertTrue(cm.removeRequestedPath("plan1"));
    assertTrue(cm.getConflicts(path).isEmpty());
    // Add multiple paths
    assertTrue(cm.addRequestedPath(path, "plan1", "veh1"));
    assertTrue(cm.addRequestedPath(path, "plan2", "veh2"));
    assertFalse(cm.getConflicts(path).isEmpty());
    // Remove one path
    assertTrue(cm.removeRequestedPath("plan1"));
    assertFalse(cm.getConflicts(path).isEmpty());
    // Remove non-added path
    assertFalse(cm.removeRequestedPath("plan1"));
    assertFalse(cm.getConflicts(path).isEmpty());
    // Remove all paths
    assertTrue(cm.removeRequestedPath("plan2"));
    assertTrue(cm.getConflicts(path).isEmpty());
    // Add duplicate vehicles
    assertTrue(cm.addRequestedPath(path, "plan1", "veh1"));
    assertTrue(cm.addRequestedPath(path, "plan1", "veh1"));
    assertFalse(cm.getConflicts(path).isEmpty());
    assertTrue(cm.removeRequestedPath("plan1"));
    assertTrue(cm.getConflicts(path).isEmpty());

    // Test null path
    assertFalse(cm.addRequestedPath(null, "plan1", "veh1"));
    // Test empty path
    assertFalse(cm.addRequestedPath(new LinkedList<>(), "plan1", "veh1"));
    // Test null string
    assertFalse(cm.addRequestedPath(path, null, "veh1"));
    assertFalse(cm.addRequestedPath(path, "plan1", null));
    assertFalse(cm.addRequestedPath(path, null, null));
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
    //// Test conflict with same path
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
    rp = new RoutePointStamped(1.0, 0, 1.0);
    rp.setSegDowntrack(1.0);
    rp.setSegment(routeMsg.getSegments().get(0));
    path.add(rp);
    rp = new RoutePointStamped(1.5, 0, 1.5);
    rp.setSegDowntrack(1.5);
    rp.setSegment(routeMsg.getSegments().get(0));
    path.add(rp);
    rp = new RoutePointStamped(2.0, 0, 2.0);
    rp.setSegDowntrack(2.0);
    rp.setSegment(routeMsg.getSegments().get(0));
    path.add(rp);


    // Add path
    assertTrue(cm.addMobilityPath(path, "veh1"));
    // Check conflict with same path
    List<ConflictSpace> conflicts = cm.getConflicts(path);
    assertEquals(1, conflicts.size());
    assertEquals(0, conflicts.get(0).getStartDowntrack(), 0.0000001);
    assertEquals(0, conflicts.get(0).getStartTime(), 0.0000001);
    assertEquals(2.0, conflicts.get(0).getEndDowntrack(), 0.0000001);
    assertEquals(2.0, conflicts.get(0).getEndTime(), 0.0000001);
    assertEquals(0, conflicts.get(0).getLane());
    assertEquals(routeMsg.getSegments().get(0), conflicts.get(0).getSegment());
    assertTrue(conflicts.get(0).getConflictingVehicles().contains("veh1"));
    assertEquals(1, conflicts.get(0).getConflictingVehicles().size());

    //// Test conflict with multiple paths
    // Add path
    assertTrue(cm.addMobilityPath(path, "veh2"));
    // Check conflict with same path
    conflicts = cm.getConflicts(path);
    assertEquals(1, conflicts.size());
    assertEquals(0, conflicts.get(0).getStartDowntrack(), 0.0000001);
    assertEquals(0, conflicts.get(0).getStartTime(), 0.0000001);
    assertEquals(2.0, conflicts.get(0).getEndDowntrack(), 0.0000001);
    assertEquals(2.0, conflicts.get(0).getEndTime(), 0.0000001);
    assertEquals(0, conflicts.get(0).getLane());
    assertEquals(routeMsg.getSegments().get(0), conflicts.get(0).getSegment());
    assertTrue(conflicts.get(0).getConflictingVehicles().contains("veh1"));
    assertTrue(conflicts.get(0).getConflictingVehicles().contains("veh2"));
    assertEquals(2, conflicts.get(0).getConflictingVehicles().size());
    
    // Remove extra path
    assertTrue(cm.removeMobilityPath("veh2"));

    //// Test no conflict in adjacent lane
    // Build path2
    List<RoutePointStamped> path2 = new ArrayList<>();
    rp = new RoutePointStamped(0, 5, 0);
    rp.setSegDowntrack(0);
    rp.setSegment(routeMsg.getSegments().get(0));
    path2.add(rp);
    rp = new RoutePointStamped(0.5, 5, 0.5);
    rp.setSegDowntrack(0.5);
    rp.setSegment(routeMsg.getSegments().get(0));
    path2.add(rp);
    rp = new RoutePointStamped(1.0, 5, 1.0);
    rp.setSegDowntrack(1.0);
    rp.setSegment(routeMsg.getSegments().get(0));
    path2.add(rp);


    // Check no conflict in adjacent lanes
    conflicts = cm.getConflicts(path2);
    assertTrue(conflicts.isEmpty());

    //// Test conflict in lane change
    // Build path2
    path2 = new ArrayList<>();
    rp = new RoutePointStamped(0, 5, 0);
    rp.setSegDowntrack(0);
    rp.setSegment(routeMsg.getSegments().get(0));
    path2.add(rp);
    rp = new RoutePointStamped(0.5, 0, 0.5);
    rp.setSegDowntrack(0.5);
    rp.setSegment(routeMsg.getSegments().get(0));
    path2.add(rp);
    rp = new RoutePointStamped(1.0, 0, 1.0);
    rp.setSegDowntrack(1.0);
    rp.setSegment(routeMsg.getSegments().get(0));
    path2.add(rp);


    // Check conflict in lane change
    conflicts = cm.getConflicts(path2);
    assertEquals(1, conflicts.size());
    assertEquals(0.5, conflicts.get(0).getStartDowntrack(), 0.0000001);
    assertEquals(0.5, conflicts.get(0).getStartTime(), 0.0000001);
    assertEquals(1.0, conflicts.get(0).getEndDowntrack(), 0.0000001);
    assertEquals(1.0, conflicts.get(0).getEndTime(), 0.0000001);
    assertEquals(0, conflicts.get(0).getLane());
    assertEquals(routeMsg.getSegments().get(0), conflicts.get(0).getSegment());
    assertTrue(conflicts.get(0).getConflictingVehicles().contains("veh1"));
    assertEquals(1, conflicts.get(0).getConflictingVehicles().size());


    //// Test split conflicts
    // Build path2
    path2 = new ArrayList<>();
    rp = new RoutePointStamped(0, 0, 0);
    rp.setSegDowntrack(0);
    rp.setSegment(routeMsg.getSegments().get(0));
    path2.add(rp);
    rp = new RoutePointStamped(0.5, 0, 0.5);
    rp.setSegDowntrack(0.5);
    rp.setSegment(routeMsg.getSegments().get(0));
    path2.add(rp);
    rp = new RoutePointStamped(1.0, 5, 1.0);
    rp.setSegDowntrack(1.0);
    rp.setSegment(routeMsg.getSegments().get(0));
    path2.add(rp);
    rp = new RoutePointStamped(1.5, 0, 1.5);
    rp.setSegDowntrack(1.5);
    rp.setSegment(routeMsg.getSegments().get(0));
    path2.add(rp);
    rp = new RoutePointStamped(2.0, 0, 2.0);
    rp.setSegDowntrack(2.0);
    rp.setSegment(routeMsg.getSegments().get(0));
    path2.add(rp);

    // Check conflict
    conflicts = cm.getConflicts(path2);
    assertEquals(2, conflicts.size());
    // First conflict
    assertEquals(0, conflicts.get(0).getStartDowntrack(), 0.0000001);
    assertEquals(0, conflicts.get(0).getStartTime(), 0.0000001);
    assertEquals(0.5, conflicts.get(0).getEndDowntrack(), 0.0000001);
    assertEquals(0.5, conflicts.get(0).getEndTime(), 0.0000001);
    assertEquals(0, conflicts.get(0).getLane());
    assertEquals(routeMsg.getSegments().get(0), conflicts.get(0).getSegment());
    assertTrue(conflicts.get(0).getConflictingVehicles().contains("veh1"));
    assertEquals(1, conflicts.get(0).getConflictingVehicles().size());
    // Second conflict
    assertEquals(1.5, conflicts.get(1).getStartDowntrack(), 0.0000001);
    assertEquals(1.5, conflicts.get(1).getStartTime(), 0.0000001);
    assertEquals(2.0, conflicts.get(1).getEndDowntrack(), 0.0000001);
    assertEquals(2.0, conflicts.get(1).getEndTime(), 0.0000001);
    assertEquals(0, conflicts.get(1).getLane());
    assertEquals(routeMsg.getSegments().get(0), conflicts.get(1).getSegment());
    assertTrue(conflicts.get(1).getConflictingVehicles().contains("veh1"));
    assertEquals(1, conflicts.get(1).getConflictingVehicles().size());

    //// Test conflict split on lane
    assertTrue(cm.removeMobilityPath("veh1"));
    // Build path
    path = new ArrayList<>();
    rp = new RoutePointStamped(0, 0, 0);
    rp.setSegDowntrack(0);
    rp.setSegment(routeMsg.getSegments().get(0));
    path.add(rp);
    rp = new RoutePointStamped(0.5, 0, 0.5);
    rp.setSegDowntrack(0.5);
    rp.setSegment(routeMsg.getSegments().get(0));
    path.add(rp);
    rp = new RoutePointStamped(1.0, -5, 1.0);
    rp.setSegDowntrack(1.0);
    rp.setSegment(routeMsg.getSegments().get(0));
    path.add(rp);
    rp = new RoutePointStamped(1.5, -5, 1.5);
    rp.setSegDowntrack(1.5);
    rp.setSegment(routeMsg.getSegments().get(0));
    path.add(rp);
    rp = new RoutePointStamped(2.0, -5, 2.0);
    rp.setSegDowntrack(2.0);
    rp.setSegment(routeMsg.getSegments().get(0));
    path.add(rp);

    // Add path
    assertTrue(cm.addMobilityPath(path, "veh1"));

    // Check conflict
    conflicts = cm.getConflicts(path);
    assertEquals(2, conflicts.size());
    // First conflict
    assertEquals(0, conflicts.get(0).getStartDowntrack(), 0.0000001);
    assertEquals(0, conflicts.get(0).getStartTime(), 0.0000001);
    assertEquals(0.5, conflicts.get(0).getEndDowntrack(), 0.0000001);
    assertEquals(0.5, conflicts.get(0).getEndTime(), 0.0000001);
    assertEquals(0, conflicts.get(0).getLane());
    assertEquals(routeMsg.getSegments().get(0), conflicts.get(0).getSegment());
    assertTrue(conflicts.get(0).getConflictingVehicles().contains("veh1"));
    assertEquals(1, conflicts.get(0).getConflictingVehicles().size());
    // Second conflict
    assertEquals(0.5, conflicts.get(1).getStartDowntrack(), 0.0000001);
    assertEquals(0.5, conflicts.get(1).getStartTime(), 0.0000001);
    assertEquals(2.0, conflicts.get(1).getEndDowntrack(), 0.0000001);
    assertEquals(2.0, conflicts.get(1).getEndTime(), 0.0000001);
    assertEquals(1, conflicts.get(1).getLane());
    assertEquals(routeMsg.getSegments().get(0), conflicts.get(1).getSegment());
    assertTrue(conflicts.get(1).getConflictingVehicles().contains("veh1"));
    assertEquals(1, conflicts.get(1).getConflictingVehicles().size());

    //// Test single point conflict
    // Build path2
    path2 = new ArrayList<>();
    rp = new RoutePointStamped(0, 0, 0);
    rp.setSegDowntrack(0);
    rp.setSegment(routeMsg.getSegments().get(0));
    path2.add(rp);

    // Check conflict
    conflicts = cm.getConflicts(path2);
    assertEquals(1, conflicts.size());
    // First conflict
    assertEquals(0, conflicts.get(0).getStartDowntrack(), 0.0000001);
    assertEquals(0, conflicts.get(0).getStartTime(), 0.0000001);
    assertEquals(0 + downtrackMargin, conflicts.get(0).getEndDowntrack(), 0.0000001);
    assertEquals(0 + timeMargin, conflicts.get(0).getEndTime(), 0.0000001);
    assertEquals(0, conflicts.get(0).getLane());
    assertEquals(routeMsg.getSegments().get(0), conflicts.get(0).getSegment());
    assertTrue(conflicts.get(0).getConflictingVehicles().contains("veh1"));
    assertEquals(1, conflicts.get(0).getConflictingVehicles().size());

    //// Test conflict with requested path
    assertTrue(cm.removeMobilityPath("veh1"));
    assertTrue(cm.getConflicts(path).isEmpty());
    // Build path
    path = new ArrayList<>();
    rp = new RoutePointStamped(0, 0, 0);
    rp.setSegDowntrack(0);
    rp.setSegment(routeMsg.getSegments().get(0));
    path.add(rp);
    rp = new RoutePointStamped(0.5, 0, 0.5);
    rp.setSegDowntrack(0.5);
    rp.setSegment(routeMsg.getSegments().get(0));
    path.add(rp);
    rp = new RoutePointStamped(1.0, -5, 1.0);
    rp.setSegDowntrack(1.0);
    rp.setSegment(routeMsg.getSegments().get(0));
    path.add(rp);
    rp = new RoutePointStamped(1.5, -5, 1.5);
    rp.setSegDowntrack(1.5);
    rp.setSegment(routeMsg.getSegments().get(0));
    path.add(rp);
    rp = new RoutePointStamped(2.0, -5, 2.0);
    rp.setSegDowntrack(2.0);
    rp.setSegment(routeMsg.getSegments().get(0));
    path.add(rp);

    // Add path as requested path
    assertTrue(cm.addRequestedPath(path, "plan1", "veh1"));

    // Check conflict
    conflicts = cm.getConflicts(path);
    assertEquals(2, conflicts.size());
    // First conflict
    assertEquals(0, conflicts.get(0).getStartDowntrack(), 0.0000001);
    assertEquals(0, conflicts.get(0).getStartTime(), 0.0000001);
    assertEquals(0.5, conflicts.get(0).getEndDowntrack(), 0.0000001);
    assertEquals(0.5, conflicts.get(0).getEndTime(), 0.0000001);
    assertEquals(0, conflicts.get(0).getLane());
    assertEquals(routeMsg.getSegments().get(0), conflicts.get(0).getSegment());
    assertTrue(conflicts.get(0).getConflictingVehicles().contains("veh1"));
    assertEquals(1, conflicts.get(0).getConflictingVehicles().size());
    // Second conflict
    assertEquals(0.5, conflicts.get(1).getStartDowntrack(), 0.0000001);
    assertEquals(0.5, conflicts.get(1).getStartTime(), 0.0000001);
    assertEquals(2.0, conflicts.get(1).getEndDowntrack(), 0.0000001);
    assertEquals(2.0, conflicts.get(1).getEndTime(), 0.0000001);
    assertEquals(1, conflicts.get(1).getLane());
    assertEquals(routeMsg.getSegments().get(0), conflicts.get(1).getSegment());
    assertTrue(conflicts.get(1).getConflictingVehicles().contains("veh1"));
    assertEquals(1, conflicts.get(1).getConflictingVehicles().size());


    //// Test null path
    assertTrue(cm.getConflicts(null).isEmpty());
    //// Test empty path
    assertTrue(cm.getConflicts(new LinkedList<>()).isEmpty());
    //// Remove path
    assertTrue(cm.removeRequestedPath("plan1"));
    assertTrue(cm.getConflicts(path).isEmpty());
    //// Test conflict with no path stored
    assertTrue(cm.getConflicts(path).isEmpty());
  }
  
  @Test
  public void testGetConflictsBetweenPaths() {
    double[] cellSize = {1,1,1};
    double downtrackMargin = 0.5;
    double crosstrackMargin = 0.5;
    double timeMargin = 0.5;
    MockTimeProvider timeProvider = new MockTimeProvider();
    timeProvider.setCurrentTime(0.0);
    ConflictManager cm = new ConflictManager(cellSize, downtrackMargin, crosstrackMargin, timeMargin, timeProvider);
    //// Test conflict with same path
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
    rp = new RoutePointStamped(1.0, 0, 1.0);
    rp.setSegDowntrack(1.0);
    rp.setSegment(routeMsg.getSegments().get(0));
    path.add(rp);
    rp = new RoutePointStamped(1.5, 0, 1.5);
    rp.setSegDowntrack(1.5);
    rp.setSegment(routeMsg.getSegments().get(0));
    path.add(rp);
    rp = new RoutePointStamped(2.0, 0, 2.0);
    rp.setSegDowntrack(2.0);
    rp.setSegment(routeMsg.getSegments().get(0));
    path.add(rp);

    // Check conflict with same path
    List<ConflictSpace> conflicts = cm.getConflicts(path, path);
    assertEquals(1, conflicts.size());
    assertEquals(0, conflicts.get(0).getStartDowntrack(), 0.0000001);
    assertEquals(0, conflicts.get(0).getStartTime(), 0.0000001);
    assertEquals(2.0, conflicts.get(0).getEndDowntrack(), 0.0000001);
    assertEquals(2.0, conflicts.get(0).getEndTime(), 0.0000001);
    assertEquals(0, conflicts.get(0).getLane());
    assertEquals(routeMsg.getSegments().get(0), conflicts.get(0).getSegment());

    //// Test no conflict in adjacent lane
    // Build path2
    List<RoutePointStamped> path2 = new ArrayList<>();
    rp = new RoutePointStamped(0, 5, 0);
    rp.setSegDowntrack(0);
    rp.setSegment(routeMsg.getSegments().get(0));
    path2.add(rp);
    rp = new RoutePointStamped(0.5, 5, 0.5);
    rp.setSegDowntrack(0.5);
    rp.setSegment(routeMsg.getSegments().get(0));
    path2.add(rp);
    rp = new RoutePointStamped(1.0, 5, 1.0);
    rp.setSegDowntrack(1.0);
    rp.setSegment(routeMsg.getSegments().get(0));
    path2.add(rp);


    // Check no conflict in adjacent lanes
    conflicts = cm.getConflicts(path2, path);
    assertTrue(conflicts.isEmpty());

    //// Test conflict in lane change
    // Build path2
    path2 = new ArrayList<>();
    rp = new RoutePointStamped(0, 5, 0);
    rp.setSegDowntrack(0);
    rp.setSegment(routeMsg.getSegments().get(0));
    path2.add(rp);
    rp = new RoutePointStamped(0.5, 0, 0.5);
    rp.setSegDowntrack(0.5);
    rp.setSegment(routeMsg.getSegments().get(0));
    path2.add(rp);
    rp = new RoutePointStamped(1.0, 0, 1.0);
    rp.setSegDowntrack(1.0);
    rp.setSegment(routeMsg.getSegments().get(0));
    path2.add(rp);


    // Check conflict in lane change
    conflicts = cm.getConflicts(path2, path);
    assertEquals(1, conflicts.size());
    assertEquals(0.5, conflicts.get(0).getStartDowntrack(), 0.0000001);
    assertEquals(0.5, conflicts.get(0).getStartTime(), 0.0000001);
    assertEquals(1.0, conflicts.get(0).getEndDowntrack(), 0.0000001);
    assertEquals(1.0, conflicts.get(0).getEndTime(), 0.0000001);
    assertEquals(0, conflicts.get(0).getLane());
    assertEquals(routeMsg.getSegments().get(0), conflicts.get(0).getSegment());


    //// Test split conflicts
    // Build path2
    path2 = new ArrayList<>();
    rp = new RoutePointStamped(0, 0, 0);
    rp.setSegDowntrack(0);
    rp.setSegment(routeMsg.getSegments().get(0));
    path2.add(rp);
    rp = new RoutePointStamped(0.5, 0, 0.5);
    rp.setSegDowntrack(0.5);
    rp.setSegment(routeMsg.getSegments().get(0));
    path2.add(rp);
    rp = new RoutePointStamped(1.0, 5, 1.0);
    rp.setSegDowntrack(1.0);
    rp.setSegment(routeMsg.getSegments().get(0));
    path2.add(rp);
    rp = new RoutePointStamped(1.5, 0, 1.5);
    rp.setSegDowntrack(1.5);
    rp.setSegment(routeMsg.getSegments().get(0));
    path2.add(rp);
    rp = new RoutePointStamped(2.0, 0, 2.0);
    rp.setSegDowntrack(2.0);
    rp.setSegment(routeMsg.getSegments().get(0));
    path2.add(rp);

    // Check conflict
    conflicts = cm.getConflicts(path2, path);
    assertEquals(2, conflicts.size());
    // First conflict
    assertEquals(0, conflicts.get(0).getStartDowntrack(), 0.0000001);
    assertEquals(0, conflicts.get(0).getStartTime(), 0.0000001);
    assertEquals(0.5, conflicts.get(0).getEndDowntrack(), 0.0000001);
    assertEquals(0.5, conflicts.get(0).getEndTime(), 0.0000001);
    assertEquals(0, conflicts.get(0).getLane());
    assertEquals(routeMsg.getSegments().get(0), conflicts.get(0).getSegment());
    // Second conflict
    assertEquals(1.5, conflicts.get(1).getStartDowntrack(), 0.0000001);
    assertEquals(1.5, conflicts.get(1).getStartTime(), 0.0000001);
    assertEquals(2.0, conflicts.get(1).getEndDowntrack(), 0.0000001);
    assertEquals(2.0, conflicts.get(1).getEndTime(), 0.0000001);
    assertEquals(0, conflicts.get(1).getLane());
    assertEquals(routeMsg.getSegments().get(0), conflicts.get(1).getSegment());

    //// Test conflict split on lane
    // Build path
    path = new ArrayList<>();
    rp = new RoutePointStamped(0, 0, 0);
    rp.setSegDowntrack(0);
    rp.setSegment(routeMsg.getSegments().get(0));
    path.add(rp);
    rp = new RoutePointStamped(0.5, 0, 0.5);
    rp.setSegDowntrack(0.5);
    rp.setSegment(routeMsg.getSegments().get(0));
    path.add(rp);
    rp = new RoutePointStamped(1.0, -5, 1.0);
    rp.setSegDowntrack(1.0);
    rp.setSegment(routeMsg.getSegments().get(0));
    path.add(rp);
    rp = new RoutePointStamped(1.5, -5, 1.5);
    rp.setSegDowntrack(1.5);
    rp.setSegment(routeMsg.getSegments().get(0));
    path.add(rp);
    rp = new RoutePointStamped(2.0, -5, 2.0);
    rp.setSegDowntrack(2.0);
    rp.setSegment(routeMsg.getSegments().get(0));
    path.add(rp);

    // Check conflict
    conflicts = cm.getConflicts(path, path);
    assertEquals(2, conflicts.size());
    // First conflict
    assertEquals(0, conflicts.get(0).getStartDowntrack(), 0.0000001);
    assertEquals(0, conflicts.get(0).getStartTime(), 0.0000001);
    assertEquals(0.5, conflicts.get(0).getEndDowntrack(), 0.0000001);
    assertEquals(0.5, conflicts.get(0).getEndTime(), 0.0000001);
    assertEquals(0, conflicts.get(0).getLane());
    assertEquals(routeMsg.getSegments().get(0), conflicts.get(0).getSegment());
    // Second conflict
    assertEquals(0.5, conflicts.get(1).getStartDowntrack(), 0.0000001);
    assertEquals(0.5, conflicts.get(1).getStartTime(), 0.0000001);
    assertEquals(2.0, conflicts.get(1).getEndDowntrack(), 0.0000001);
    assertEquals(2.0, conflicts.get(1).getEndTime(), 0.0000001);
    assertEquals(1, conflicts.get(1).getLane());
    assertEquals(routeMsg.getSegments().get(0), conflicts.get(1).getSegment());

    //// Test single point conflict
    // Build path2
    path2 = new ArrayList<>();
    rp = new RoutePointStamped(0, 0, 0);
    rp.setSegDowntrack(0);
    rp.setSegment(routeMsg.getSegments().get(0));
    path2.add(rp);

    // Check conflict
    conflicts = cm.getConflicts(path2, path);
    assertEquals(1, conflicts.size());
    // First conflict
    assertEquals(0, conflicts.get(0).getStartDowntrack(), 0.0000001);
    assertEquals(0, conflicts.get(0).getStartTime(), 0.0000001);
    assertEquals(0 + downtrackMargin, conflicts.get(0).getEndDowntrack(), 0.0000001);
    assertEquals(0 + timeMargin, conflicts.get(0).getEndTime(), 0.0000001);
    assertEquals(0, conflicts.get(0).getLane());
    assertEquals(routeMsg.getSegments().get(0), conflicts.get(0).getSegment());

    //// Test null path
    assertTrue(cm.getConflicts(null, path).isEmpty());
    assertTrue(cm.getConflicts(path, null).isEmpty());
    assertTrue(cm.getConflicts(null, null).isEmpty());
    //// Test empty path
    assertTrue(cm.getConflicts(new LinkedList<>(), path).isEmpty());
    assertTrue(cm.getConflicts(path, new LinkedList<>()).isEmpty());
    assertTrue(cm.getConflicts(new LinkedList<>(), new LinkedList<>()).isEmpty());
  }

  @Test
  public void testTimeFiltering() {
    double[] cellSize = {1,1,1};
    double downtrackMargin = 0.5;
    double crosstrackMargin = 0.5;
    double timeMargin = 0.5;
    MockTimeProvider timeProvider = new MockTimeProvider();
    timeProvider.setCurrentTime(0.0);
    ConflictManager cm = new ConflictManager(cellSize, downtrackMargin, crosstrackMargin, timeMargin, timeProvider);
    //// Test conflict with same path
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
    rp = new RoutePointStamped(1.0, 0, 1.0);
    rp.setSegDowntrack(1.0);
    rp.setSegment(routeMsg.getSegments().get(0));
    path.add(rp);
    rp = new RoutePointStamped(1.5, 0, 1.5);
    rp.setSegDowntrack(1.5);
    rp.setSegment(routeMsg.getSegments().get(0));
    path.add(rp);
    rp = new RoutePointStamped(2.0, 0, 2.0);
    rp.setSegDowntrack(2.0);
    rp.setSegment(routeMsg.getSegments().get(0));
    path.add(rp);


    // Add path
    assertTrue(cm.addMobilityPath(path, "veh1"));
    // Check conflict with same path
    List<ConflictSpace> conflicts = cm.getConflicts(path);
    assertEquals(1, conflicts.size());
    assertEquals(0, conflicts.get(0).getStartDowntrack(), 0.0000001);
    assertEquals(0, conflicts.get(0).getStartTime(), 0.0000001);
    assertEquals(2.0, conflicts.get(0).getEndDowntrack(), 0.0000001);
    assertEquals(2.0, conflicts.get(0).getEndTime(), 0.0000001);
    assertEquals(0, conflicts.get(0).getLane());
    assertEquals(routeMsg.getSegments().get(0), conflicts.get(0).getSegment());
    assertTrue(conflicts.get(0).getConflictingVehicles().contains("veh1"));
    assertEquals(1, conflicts.get(0).getConflictingVehicles().size());

    //// Test time filtering
    // Change time
    timeProvider.setCurrentTime(1000);

    List<RoutePointStamped> latePath = new ArrayList<>();
    rp = new RoutePointStamped(0, 0, 1000);
    rp.setSegDowntrack(0);
    rp.setSegment(routeMsg.getSegments().get(0));
    latePath.add(rp);
    rp = new RoutePointStamped(0.5, 0, 1000.5);
    rp.setSegDowntrack(0.5);
    rp.setSegment(routeMsg.getSegments().get(0));
    latePath.add(rp);
    rp = new RoutePointStamped(1.0, 0, 1001.0);
    rp.setSegDowntrack(1.0);
    rp.setSegment(routeMsg.getSegments().get(0));
    latePath.add(rp);
    rp = new RoutePointStamped(1.5, 0, 1001.5);
    rp.setSegDowntrack(1.5);
    rp.setSegment(routeMsg.getSegments().get(0));
    latePath.add(rp);
    rp = new RoutePointStamped(2.0, 0, 1002.0);
    rp.setSegDowntrack(2.0);
    rp.setSegment(routeMsg.getSegments().get(0));
    latePath.add(rp);


    conflicts = cm.getConflicts(latePath);
    assertTrue(conflicts.isEmpty());
    assertFalse(cm.removeMobilityPath("veh1"));

    // Re-add path and reset time
    timeProvider.setCurrentTime(0.0);
    assertTrue(cm.addMobilityPath(path, "veh1"));

    //// Test time filtering with multiple paths
    // Add paths
    assertTrue(cm.addMobilityPath(path, "veh2"));
    assertTrue(cm.addMobilityPath(path, "veh3"));
    // Check conflict with same path
    conflicts = cm.getConflicts(path);
    assertEquals(1, conflicts.size());
    assertEquals(0, conflicts.get(0).getStartDowntrack(), 0.0000001);
    assertEquals(0, conflicts.get(0).getStartTime(), 0.0000001);
    assertEquals(2.0, conflicts.get(0).getEndDowntrack(), 0.0000001);
    assertEquals(2.0, conflicts.get(0).getEndTime(), 0.0000001);
    assertEquals(0, conflicts.get(0).getLane());
    assertEquals(routeMsg.getSegments().get(0), conflicts.get(0).getSegment());
    assertTrue(conflicts.get(0).getConflictingVehicles().contains("veh1"));
    assertTrue(conflicts.get(0).getConflictingVehicles().contains("veh2"));
    assertTrue(conflicts.get(0).getConflictingVehicles().contains("veh3"));
    assertEquals(3, conflicts.get(0).getConflictingVehicles().size());
    
    timeProvider.setCurrentTime(1000);
    assertTrue(cm.addMobilityPath(latePath, "veh4"));

    conflicts = cm.getConflicts(latePath);
    assertEquals(1, conflicts.size());
    assertEquals(0, conflicts.get(0).getStartDowntrack(), 0.0000001);
    assertEquals(1000, conflicts.get(0).getStartTime(), 0.0000001);
    assertEquals(2.0, conflicts.get(0).getEndDowntrack(), 0.0000001);
    assertEquals(1002.0, conflicts.get(0).getEndTime(), 0.0000001);
    assertEquals(0, conflicts.get(0).getLane());
    assertEquals(routeMsg.getSegments().get(0), conflicts.get(0).getSegment());
    assertTrue(conflicts.get(0).getConflictingVehicles().contains("veh4"));
    assertEquals(1, conflicts.get(0).getConflictingVehicles().size());

    conflicts = cm.getConflicts(path);
    assertTrue(conflicts.isEmpty());

    // Change time
    timeProvider.setCurrentTime(2000);
    // Check if now old path was deleted
    conflicts = cm.getConflicts(latePath);
    assertTrue(conflicts.isEmpty());

  }
}
