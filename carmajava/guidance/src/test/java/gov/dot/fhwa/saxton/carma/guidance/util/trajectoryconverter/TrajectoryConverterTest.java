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

package gov.dot.fhwa.saxton.carma.guidance.util.trajectoryconverter;

import java.util.List;
import java.math.BigDecimal;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedList;
import org.apache.commons.logging.Log;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import org.ros.message.MessageFactory;
import org.ros.node.NodeConfiguration;
import org.ros.rosjava_geometry.Transform;
import org.ros.rosjava_geometry.Vector3;

import cav_msgs.LocationECEF;
import cav_msgs.LocationOffsetECEF;

import static org.junit.Assert.*;
import static org.mockito.Mockito.*;

import gov.dot.fhwa.saxton.carma.geometry.GeodesicCartesianConverter;
import gov.dot.fhwa.saxton.carma.geometry.cartesian.Point;
import gov.dot.fhwa.saxton.carma.geometry.cartesian.Point3D;
import gov.dot.fhwa.saxton.carma.geometry.geodesic.Location;
import gov.dot.fhwa.saxton.carma.guidance.conflictdetector.ConflictManager;
import gov.dot.fhwa.saxton.carma.guidance.conflictdetector.IMobilityTimeProvider;
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
import gov.dot.fhwa.saxton.carma.route.FileStrategy;
import gov.dot.fhwa.saxton.carma.route.Route;
import gov.dot.fhwa.saxton.carma.route.RouteSegment;

public class TrajectoryConverterTest {

  private IPlugin mockPlugin;
  private Route route;


  private Log log;
  private NodeConfiguration nodeConfiguration = NodeConfiguration.newPrivate();
  private MessageFactory messageFactory = nodeConfiguration.getTopicMessageFactory();
  private GeodesicCartesianConverter gcc = new GeodesicCartesianConverter();

  @Before
  public void setup() {
    ILoggerFactory mockFact = mock(ILoggerFactory.class);
    ILogger mockLogger = mock(ILogger.class);
    when(mockFact.createLoggerForClass(any())).thenReturn(mockLogger);
    LoggerManager.setLoggerFactory(mockFact);
    mockPlugin = mock(IPlugin.class);
    log = mock(Log.class);
    route = (new FileStrategy("../route/src/test/resources/routes/colonial_farm_rd_outbound.yaml", log)).load();
    route = Route.fromMessage(route.toMessage(messageFactory)); // Assign waypoint ids
  }

  @Test
  public void testMessageToPath() {
      route = (new FileStrategy("../route/src/test/resources/routes/tfhrc_circle.yaml", log)).load();
      final int MAX_POINTS_IN_PATH = 60;
      final double TIME_STEP = 0.1;
      TrajectoryConverter tc = new TrajectoryConverter(MAX_POINTS_IN_PATH, TIME_STEP, messageFactory);
      tc.setRoute(route);
      cav_msgs.Trajectory message = mock(cav_msgs.Trajectory.class);
      cav_msgs.LocationECEF ecef = mock(LocationECEF.class);
      when(ecef.getEcefX()).thenReturn(110449015);
      when(ecef.getEcefY()).thenReturn(-484206432);
      when(ecef.getEcefZ()).thenReturn(398858656);
      when(ecef.getTimestamp()).thenReturn(System.currentTimeMillis());
      when(message.getLocation()).thenReturn(ecef);
      List<LocationOffsetECEF> offsets = new LinkedList<>();
      LocationOffsetECEF offset1 = mock(LocationOffsetECEF.class);
      when(offset1.getOffsetX()).thenReturn((short) -15);
      when(offset1.getOffsetY()).thenReturn((short) -7);
      when(offset1.getOffsetZ()).thenReturn((short) -5);
      offsets.add(offset1);
      when(message.getOffsets()).thenReturn(offsets);
      long startTime = System.currentTimeMillis();
      List<RoutePointStamped> resultPoints = tc.messageToPath(message);
      long endTime = System.currentTimeMillis();
      System.out.println("\n\n\n PathSize: " + resultPoints.size() + "\n MS: " + (endTime - startTime) + "\n\n\n");
      for(RoutePointStamped rps : resultPoints) {
          System.out.println(BigDecimal.valueOf(rps.getDowntrack()).toPlainString());
          System.out.println(BigDecimal.valueOf(rps.getCrosstrack()).toPlainString());
          System.out.println(BigDecimal.valueOf(rps.getStamp()).toPlainString());
          System.out.println(rps.getSegDowntrack());
          System.out.println(rps.getSegmentIdx());
      }
  }
  
  @Test
  public void testConvertToPath() {
    final int MAX_POINTS_IN_PATH = 1000;
    final double TIME_STEP = 0.1;
    TrajectoryConverter tc = new TrajectoryConverter(MAX_POINTS_IN_PATH, TIME_STEP, messageFactory);
    tc.setRoute(route);
    List<ECEFPointStamped> ecefPath = new LinkedList<>();

    // Setup starting configuration
    // Vehicle at very start of route

    // Build trajectory
    // SpeedUp dist: [0,40) speed: [0,11] -> Slow down dist: [40,50) speed [15,10]-> Stead speed dist: [50,100) speed: [10,10]
    
    LongitudinalManeuver speedUp = mock(SpeedUp.class);
    when(speedUp.getStartSpeed()).thenReturn(0.0);
    when(speedUp.getTargetSpeed()).thenReturn(15.0);
    when(speedUp.getStartDistance()).thenReturn(0.0);
    when(speedUp.getEndDistance()).thenReturn(40.0);

    LongitudinalManeuver slowDown = mock(SlowDown.class);
    when(slowDown.getStartSpeed()).thenReturn(15.0);
    when(slowDown.getTargetSpeed()).thenReturn(10.0);
    when(slowDown.getStartDistance()).thenReturn(40.0);
    when(slowDown.getEndDistance()).thenReturn(50.0);
    
    LongitudinalManeuver steadySpeed = mock(SteadySpeed.class);
    when(steadySpeed.getStartSpeed()).thenReturn(10.0);
    when(steadySpeed.getTargetSpeed()).thenReturn(10.0);
    when(steadySpeed.getStartDistance()).thenReturn(50.0);
    when(steadySpeed.getEndDistance()).thenReturn(100.0);

    LaneKeeping laneKeeping = mock(LaneKeeping.class);
    when(laneKeeping.getEndingRelativeLane()).thenReturn(0);
    when(laneKeeping.getStartDistance()).thenReturn(0.0);
    when(laneKeeping.getEndDistance()).thenReturn(40.0);

    LaneChange laneChange1 = mock(LaneChange.class);
    when(laneChange1.getEndingRelativeLane()).thenReturn(1);
    when(laneChange1.getStartDistance()).thenReturn(40.0);
    when(laneChange1.getEndDistance()).thenReturn(70.0);

    FutureLateralManeuver futureLaneChange1 = mock(FutureLateralManeuver.class);
    when(futureLaneChange1.getEndingRelativeLane()).thenReturn(1);
    when(futureLaneChange1.getStartDistance()).thenReturn(40.0);
    when(futureLaneChange1.getEndDistance()).thenReturn(70.0);
    when(futureLaneChange1.getLateralManeuvers()).thenReturn(new LinkedList<>(Arrays.asList(laneChange1)));  

    FutureLateralManeuver futureLaneChange2 = mock(FutureLateralManeuver.class);
    when(futureLaneChange2.getEndingRelativeLane()).thenReturn(-1);
    when(futureLaneChange2.getStartDistance()).thenReturn(70.0);
    when(futureLaneChange2.getEndDistance()).thenReturn(100.0);

    Trajectory traj = new Trajectory(0, 100.0);
    traj.addManeuver(speedUp);
    traj.addManeuver(slowDown);
    traj.addManeuver(steadySpeed);
    traj.addManeuver(laneKeeping);
    traj.addManeuver(futureLaneChange1);
    traj.addManeuver(futureLaneChange2);

    IMobilityTimeProvider timeProvider = mock(IMobilityTimeProvider.class);
    when(timeProvider.getCurrentTimeMillis()).thenReturn(0L);
    when(timeProvider.getCurrentTimeSeconds()).thenReturn(0.0);
    double[] cellSize = {5.0,2.5,0.15};
    ConflictManager cm = new ConflictManager(cellSize, 2.5, 1.0, 0.05, timeProvider);
    cm.setRoute(route);
    // Call function
    List<RoutePointStamped> tempPoints = tc.convertToPath(traj, 0, 0, 0, 0, 0, 0);
    cm.addRequestedPath(tempPoints, "planId", "vehid");
    cm.getConflicts(tempPoints, tempPoints);
    cav_msgs.Trajectory message = tc.pathToMessage(tempPoints);
    long startTime = System.currentTimeMillis();
    List<RoutePointStamped> resultPoints = tc.messageToPath(message);
    long endTime = System.currentTimeMillis();
    System.out.println("\n\n\n PathSize: " + resultPoints.size() + "\n MS: " + (endTime - startTime) + "\n\n\n");
    ecefPath = tc.toECEFPoints(tc.convertToPath(traj, 0, 0, 0, 0, 0, 0));
    assertEquals(112, ecefPath.size());

    // Check starting point
    Location expectedPoint = new Location(38.95647, -77.15031, 72.0);
    Point3D expecedfECEFPoint = gcc.geodesic2Cartesian(expectedPoint, Transform.identity());
    assertTrue(expecedfECEFPoint.almostEquals(ecefPath.get(0).getPoint(), 0.1));
    assertEquals(0, ecefPath.get(0).getStamp(), 0.0001);
    // Check ending point
    expectedPoint = new Location(38.95594, -77.15114, 72.0);
    expecedfECEFPoint = gcc.geodesic2Cartesian(expectedPoint, Transform.identity());
    assertTrue(expecedfECEFPoint.almostEquals(ecefPath.get(ecefPath.size() - 1).getPoint(), 0.5));
    assertEquals(11.1, ecefPath.get(ecefPath.size() - 1).getStamp(), 0.1);

    // System.out.println("\n\n");
    // int count = 0;
    // for(Point3DStamped p: path) {
    //   if(count % 2 == 0) {
    //     System.out.println(gcc.cartesian2Geodesic(p.getPoint(), Transform.identity()));
    //   }
    //   count++;
    // }
    // System.out.println("\n\n");
  }

  @Test
  public void testAddLongitudinalManeuverToPath() {
    final int MAX_POINTS_IN_PATH = 20;
    final double TIME_STEP = 1;
    TrajectoryConverter tc = new TrajectoryConverter(MAX_POINTS_IN_PATH, TIME_STEP, messageFactory);
    tc.setRoute(route);
    List<RoutePointStamped> path = new LinkedList<>();
    List<ECEFPointStamped> pathECEF = new LinkedList<>();
    
    // Test Steady Speed Maneuver at very start
    // Vehicle is at very start of the route going 10m/s for the next 100 m
    LongitudinalSimulationData startingData = new LongitudinalSimulationData(0, 0, 0, 0);
    LongitudinalSimulationData endingData = new LongitudinalSimulationData(10, 100, 24.384, 3);

    LongitudinalManeuver steadySpeed = mock(SteadySpeed.class);
    when(steadySpeed.getStartSpeed()).thenReturn(10.0);
    when(steadySpeed.getTargetSpeed()).thenReturn(10.0);
    when(steadySpeed.getStartDistance()).thenReturn(startingData.downtrack);
    when(steadySpeed.getEndDistance()).thenReturn(endingData.downtrack);

    // Call function
    LongitudinalSimulationData result = tc.addLongitudinalManeuverToPath(steadySpeed, path, startingData);
    assertTrue(endingData.almostEquals(result, 0.01));

    // Check number of points
    assertEquals(11, path.size());
    // Put points in ecef
    pathECEF = tc.toECEFPoints(path);

    // Check starting point
    Location expectedPoint = new Location(38.95647, -77.15031, 72.0);
    Point3D expecedfECEFPoint = gcc.geodesic2Cartesian(expectedPoint, Transform.identity());
    assertTrue(expecedfECEFPoint.almostEquals(pathECEF.get(0).getPoint(), 0.1));
    assertEquals(startingData.simTime, pathECEF.get(0).getStamp(), 0.0001);
    // Check ending point
    expectedPoint = new Location(38.95594, -77.15114, 72.0);
    expecedfECEFPoint = gcc.geodesic2Cartesian(expectedPoint, Transform.identity());
    assertTrue(expecedfECEFPoint.almostEquals(pathECEF.get(pathECEF.size() - 1).getPoint(), 0.5));
    assertEquals(endingData.simTime, pathECEF.get(pathECEF.size() - 1).getStamp(), 0.0001);
    
    // Test Steady Speed Maneuver at different point along route in the middle of the maneuver
    // Vehicle is at 50m going 10m/s for next 50 m
    path = new LinkedList<>();

    startingData = new LongitudinalSimulationData(5.0, 50, 24.07, 2);
    endingData = new LongitudinalSimulationData(10, 100, 24.384, 3);

    steadySpeed = mock(SteadySpeed.class);
    when(steadySpeed.getStartSpeed()).thenReturn(10.0);
    when(steadySpeed.getTargetSpeed()).thenReturn(10.0);
    when(steadySpeed.getStartDistance()).thenReturn(40.0);
    when(steadySpeed.getEndDistance()).thenReturn(endingData.downtrack);

    // Call function
    result = tc.addLongitudinalManeuverToPath(steadySpeed, path, startingData);
    assertTrue(endingData.almostEquals(result, 0.01));

    // Check number of points
    assertEquals(6, path.size());
    // Put points in ecef
    pathECEF = tc.toECEFPoints(path);

    // Check starting point
    expectedPoint = new Location(38.95621, -77.15073, 72.0);
    expecedfECEFPoint = gcc.geodesic2Cartesian(expectedPoint, Transform.identity());
    assertTrue(expecedfECEFPoint.almostEquals(pathECEF.get(0).getPoint(), 0.5));
    assertEquals(startingData.simTime, pathECEF.get(0).getStamp(), 0.0001);
    // Check ending point
    expectedPoint = new Location(38.95594, -77.15114, 72.0);
    expecedfECEFPoint = gcc.geodesic2Cartesian(expectedPoint, Transform.identity());
    assertTrue(expecedfECEFPoint.almostEquals(pathECEF.get(pathECEF.size() - 1).getPoint(), 0.5));
    assertEquals(endingData.simTime, pathECEF.get(pathECEF.size() - 1).getStamp(), 0.0001);

    // Test Speed Up Maneuver at route start
    // Vehicle is at very start of the route accelerating from 0 m/s to 10 m/s for the next 100 m
    path = new LinkedList<>();

    startingData = new LongitudinalSimulationData(0, 0, 0, 0);
    endingData = new LongitudinalSimulationData(20, 100, 24.384, 3);

    LongitudinalManeuver speedUp = mock(SpeedUp.class);
    when(speedUp.getStartSpeed()).thenReturn(0.0);
    when(speedUp.getTargetSpeed()).thenReturn(10.0);
    when(speedUp.getStartDistance()).thenReturn(startingData.downtrack);
    when(speedUp.getEndDistance()).thenReturn(endingData.downtrack);

    // Call function
    result = tc.addLongitudinalManeuverToPath(speedUp, path, startingData);
    assertTrue(endingData.almostEquals(result, 0.01));

    // Check number of points
    assertEquals(21, path.size());
    // Put points in ecef
    pathECEF = tc.toECEFPoints(path);

    // Check starting point
    expectedPoint = new Location(38.95647, -77.15031, 72.0);
    expecedfECEFPoint = gcc.geodesic2Cartesian(expectedPoint, Transform.identity());
    assertTrue(expecedfECEFPoint.almostEquals(pathECEF.get(0).getPoint(), 0.1));
    assertEquals(startingData.simTime, pathECEF.get(0).getStamp(), 0.0001);
    // Check ending point
    expectedPoint = new Location(38.95594, -77.15114, 72.0);
    expecedfECEFPoint = gcc.geodesic2Cartesian(expectedPoint, Transform.identity());
    assertTrue(expecedfECEFPoint.almostEquals(pathECEF.get(pathECEF.size() - 1).getPoint(), 0.5));
    assertEquals(endingData.simTime, pathECEF.get(pathECEF.size() - 1).getStamp(), 0.0001);


    // Test Slow Down Maneuver at route start
    // Vehicle is at very start of the route decelerating from 10 m/s to 0 m/s for the next 100 m
    path = new LinkedList<>();

    startingData = new LongitudinalSimulationData(0, 0, 0, 0);
    endingData = new LongitudinalSimulationData(20, 100, 24.384, 3);

    LongitudinalManeuver slowDown = mock(SlowDown.class);
    when(slowDown.getStartSpeed()).thenReturn(10.0);
    when(slowDown.getTargetSpeed()).thenReturn(0.0);
    when(slowDown.getStartDistance()).thenReturn(startingData.downtrack);
    when(slowDown.getEndDistance()).thenReturn(endingData.downtrack);

    // Call function
    result = tc.addLongitudinalManeuverToPath(slowDown, path, startingData);
    assertTrue(endingData.almostEquals(result, 0.01));

    // Check number of points
    assertEquals(21, path.size());
    // Put points in ecef
    pathECEF = tc.toECEFPoints(path);

    // Check starting point
    expectedPoint = new Location(38.95647, -77.15031, 72.0);
    expecedfECEFPoint = gcc.geodesic2Cartesian(expectedPoint, Transform.identity());
    assertTrue(expecedfECEFPoint.almostEquals(pathECEF.get(0).getPoint(), 0.1));
    assertEquals(startingData.simTime, pathECEF.get(0).getStamp(), 0.0001);
    // Check ending point
    expectedPoint = new Location(38.95594, -77.15114, 72.0);
    expecedfECEFPoint = gcc.geodesic2Cartesian(expectedPoint, Transform.identity());
    assertTrue(expecedfECEFPoint.almostEquals(pathECEF.get(pathECEF.size() - 1).getPoint(), 0.5));
    assertEquals(endingData.simTime, pathECEF.get(pathECEF.size() - 1).getStamp(), 0.0001);
  }
}
