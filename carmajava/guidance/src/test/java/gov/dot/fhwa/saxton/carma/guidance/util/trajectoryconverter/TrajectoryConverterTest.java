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
import java.util.ArrayList;
import java.util.LinkedList;
import org.apache.commons.logging.Log;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import org.ros.message.MessageFactory;
import org.ros.node.NodeConfiguration;
import org.ros.rosjava_geometry.Transform;

import static org.junit.Assert.*;
import static org.mockito.Mockito.*;

import gov.dot.fhwa.saxton.carma.geometry.GeodesicCartesianConverter;
import gov.dot.fhwa.saxton.carma.geometry.cartesian.Point;
import gov.dot.fhwa.saxton.carma.geometry.cartesian.Point3D;
import gov.dot.fhwa.saxton.carma.geometry.geodesic.Location;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.IManeuver;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.ISimpleManeuver;
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
  public void testConvertToPath() {
    final int MAX_POINTS_IN_PATH = 1000;
    final double TIME_STEP = 0.1;
    TrajectoryConverter tc = new TrajectoryConverter(MAX_POINTS_IN_PATH, TIME_STEP);

    List<Point3DStamped> path = new LinkedList<>();

    // Setup starting configuration
    // Vehicle at very start of route
    cav_msgs.RouteState routeState = messageFactory.newFromType(cav_msgs.RouteState._TYPE);

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

    Trajectory traj = new Trajectory(0, 100.0);
    traj.addManeuver(speedUp);
    traj.addManeuver(slowDown);
    traj.addManeuver(steadySpeed);

    // Call function
    path = tc.convertToPath(traj, 0.0, routeMsg, routeState);
    assertEquals(112, path.size());

    // Check starting point
    Location expectedPoint = new Location(38.95647, -77.15031, 72.0);
    Point3D expecedfECEFPoint = gcc.geodesic2Cartesian(expectedPoint, Transform.identity());
    assertTrue(expecedfECEFPoint.almostEquals(path.get(0).getPoint(), 0.1));
    assertEquals(0, path.get(0).getStamp(), 0.0001);
    // Check ending point
    expectedPoint = new Location(38.95594, -77.15114, 72.0);
    expecedfECEFPoint = gcc.geodesic2Cartesian(expectedPoint, Transform.identity());
    assertTrue(expecedfECEFPoint.almostEquals(path.get(path.size() - 1).getPoint(), 0.5));
    assertEquals(11.1, path.get(path.size() - 1).getStamp(), 0.1);
  }

  @Test
  public void testAddLongitudinalManeuverToPath() {
    final int MAX_POINTS_IN_PATH = 20;
    final double TIME_STEP = 1;
    TrajectoryConverter tc = new TrajectoryConverter(MAX_POINTS_IN_PATH, TIME_STEP);

    List<Point3DStamped> path = new LinkedList<>();
    
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
    LongitudinalSimulationData result = tc.addLongitudinalManeuverToPath(steadySpeed, path, startingData, routeMsg);
    assertTrue(endingData.almostEquals(result, 0.01));

    // Check number of points
    assertEquals(11, path.size());

    // Check starting point
    Location expectedPoint = new Location(38.95647, -77.15031, 72.0);
    Point3D expecedfECEFPoint = gcc.geodesic2Cartesian(expectedPoint, Transform.identity());
    assertTrue(expecedfECEFPoint.almostEquals(path.get(0).getPoint(), 0.1));
    assertEquals(startingData.simTime, path.get(0).getStamp(), 0.0001);
    // Check ending point
    expectedPoint = new Location(38.95594, -77.15114, 72.0);
    expecedfECEFPoint = gcc.geodesic2Cartesian(expectedPoint, Transform.identity());
    assertTrue(expecedfECEFPoint.almostEquals(path.get(path.size() - 1).getPoint(), 0.5));
    assertEquals(endingData.simTime, path.get(path.size() - 1).getStamp(), 0.0001);
    
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
    result = tc.addLongitudinalManeuverToPath(steadySpeed, path, startingData, routeMsg);
    assertTrue(endingData.almostEquals(result, 0.01));

    // Check number of points
    assertEquals(6, path.size());

    // Check starting point
    expectedPoint = new Location(38.95621, -77.15073, 72.0);
    expecedfECEFPoint = gcc.geodesic2Cartesian(expectedPoint, Transform.identity());
    assertTrue(expecedfECEFPoint.almostEquals(path.get(0).getPoint(), 0.5));
    assertEquals(startingData.simTime, path.get(0).getStamp(), 0.0001);
    // Check ending point
    expectedPoint = new Location(38.95594, -77.15114, 72.0);
    expecedfECEFPoint = gcc.geodesic2Cartesian(expectedPoint, Transform.identity());
    assertTrue(expecedfECEFPoint.almostEquals(path.get(path.size() - 1).getPoint(), 0.5));
    assertEquals(endingData.simTime, path.get(path.size() - 1).getStamp(), 0.0001);

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
    result = tc.addLongitudinalManeuverToPath(speedUp, path, startingData, routeMsg);
    assertTrue(endingData.almostEquals(result, 0.01));

    // Check number of points
    assertEquals(21, path.size());

    // Check starting point
    expectedPoint = new Location(38.95647, -77.15031, 72.0);
    expecedfECEFPoint = gcc.geodesic2Cartesian(expectedPoint, Transform.identity());
    assertTrue(expecedfECEFPoint.almostEquals(path.get(0).getPoint(), 0.1));
    assertEquals(startingData.simTime, path.get(0).getStamp(), 0.0001);
    // Check ending point
    expectedPoint = new Location(38.95594, -77.15114, 72.0);
    expecedfECEFPoint = gcc.geodesic2Cartesian(expectedPoint, Transform.identity());
    assertTrue(expecedfECEFPoint.almostEquals(path.get(path.size() - 1).getPoint(), 0.5));
    assertEquals(endingData.simTime, path.get(path.size() - 1).getStamp(), 0.0001);


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
    result = tc.addLongitudinalManeuverToPath(slowDown, path, startingData, routeMsg);
    assertTrue(endingData.almostEquals(result, 0.01));

    // Check number of points
    assertEquals(21, path.size());

    // Check starting point
    expectedPoint = new Location(38.95647, -77.15031, 72.0);
    expecedfECEFPoint = gcc.geodesic2Cartesian(expectedPoint, Transform.identity());
    assertTrue(expecedfECEFPoint.almostEquals(path.get(0).getPoint(), 0.1));
    assertEquals(startingData.simTime, path.get(0).getStamp(), 0.0001);
    // Check ending point
    expectedPoint = new Location(38.95594, -77.15114, 72.0);
    expecedfECEFPoint = gcc.geodesic2Cartesian(expectedPoint, Transform.identity());
    assertTrue(expecedfECEFPoint.almostEquals(path.get(path.size() - 1).getPoint(), 0.5));
    assertEquals(endingData.simTime, path.get(path.size() - 1).getStamp(), 0.0001);
  }
}
