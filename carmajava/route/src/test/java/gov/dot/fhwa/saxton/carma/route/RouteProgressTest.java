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

package gov.dot.fhwa.saxton.carma.route;

import cav_msgs.SystemAlert;
import cav_srvs.AbortActiveRouteResponse;
import cav_srvs.SetActiveRouteResponse;
import cav_srvs.StartActiveRouteResponse;
import gov.dot.fhwa.saxton.carma.geometry.GeodesicCartesianConverter;
import gov.dot.fhwa.saxton.carma.geometry.cartesian.Point3D;
import gov.dot.fhwa.saxton.carma.geometry.geodesic.Location;
import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import org.ros.message.MessageFactory;
import org.ros.node.NodeConfiguration;
import org.ros.rosjava_geometry.Quaternion;
import org.ros.rosjava_geometry.Transform;
import org.ros.rosjava_geometry.Vector3;

import sensor_msgs.NavSatFix;
import sensor_msgs.NavSatStatus;

import static org.junit.Assert.assertTrue;

/**
 * Runs unit tests for the RouteWorker class
 */
public class RouteProgressTest {

  Log log;
  NodeConfiguration nodeConfiguration = NodeConfiguration.newPrivate();
  MessageFactory messageFactory = nodeConfiguration.getTopicMessageFactory();
  GeodesicCartesianConverter gcc = new GeodesicCartesianConverter();

  @Before
  public void setUp() throws Exception {
    log = LogFactory.getLog(RouteProgressTest.class);
    log.info("Setting up tests for RouteProgressFunctions");
  }

  @After
  public void tearDown() throws Exception {
  }

  /**
   * Tests the movement of a host vehicle along a route
   * @throws Exception
   */
  @Test
  public void testRouteProgress() throws Exception {
    MockRouteManager routeMgr = new MockRouteManager();
    RouteWorker routeWorker = new RouteWorker(routeMgr, log, "src/test/resources/routes/", 3, "earth", "host_vehicle");

    routeWorker.handleSystemAlertMsg(MockRouteManager.buildSystemAlert(SystemAlert.DRIVERS_READY, ""));
    assertTrue(routeWorker.systemOkay == true);

    // Test vehicle has been placed just before start of route
    Location navSatFix = new Location(38.95649, -77.15028, 72.0);
    routeMgr.setEarthToHostTransform(transformFromLocation(navSatFix));
    routeWorker.loop();

    assertTrue(routeWorker.getCurrentState() == WorkerState.ROUTE_SELECTION);

    byte response = routeWorker.setActiveRoute("Colonial Farm Rd. Outbound");
    assertTrue(response == SetActiveRouteResponse.NO_ERROR);
    assertTrue(routeWorker.getCurrentState() == WorkerState.WAITING_TO_START);

    response = routeWorker.startActiveRoute();
    assertTrue(response == StartActiveRouteResponse.NO_ERROR);

    assertTrue(routeWorker.currentSegmentIndex == 0); // start of route

    assertTrue(routeWorker.currentSegment.getUptrackWaypoint().getLocation().almostEqual(
      new Location(38.95649, -77.15028, 72.0), 0.0000001, 0.0000001));
    assertTrue(routeWorker.getCurrentState() == WorkerState.FOLLOWING_ROUTE);

    // No change in location
    routeMgr.setEarthToHostTransform(transformFromLocation(navSatFix));
    routeWorker.loop();
    assertTrue(routeWorker.currentSegmentIndex == 0); // start of route
    assertTrue(routeWorker.currentSegment.getUptrackWaypoint().getLocation().almostEqual(
      new Location(38.95649, -77.15028, 72.0), 0.0000001, 0.0000001));
    assertTrue(routeWorker.getCurrentState() == WorkerState.FOLLOWING_ROUTE);

    // Test vehicle has been placed after route file starting point
    navSatFix = new Location(38.9564, -77.15035, 72.0);
    routeMgr.setEarthToHostTransform(transformFromLocation(navSatFix));
    routeWorker.loop();

    assertTrue(routeWorker.currentSegmentIndex == 1); // second segment
    assertTrue(routeWorker.getCurrentState() == WorkerState.FOLLOWING_ROUTE);

    // Test vehicle has been placed after next point
    navSatFix = new Location(38.95631, -77.15042, 72.0);
    routeMgr.setEarthToHostTransform(transformFromLocation(navSatFix));
    routeWorker.loop();

    assertTrue(routeWorker.currentSegmentIndex == 2); // third segment
    assertTrue(routeWorker.getCurrentState() == WorkerState.FOLLOWING_ROUTE);

    // Test vehicle has been placed after end of route
    navSatFix = new Location(38.95218, -77.15189, 72.0);
    routeMgr.setEarthToHostTransform(transformFromLocation(navSatFix));
    routeWorker.loop();

    assertTrue(routeWorker.currentSegment == routeWorker.activeRoute.getLastSegment()); // third segment
    assertTrue(routeWorker.getCurrentState() == WorkerState.ROUTE_SELECTION); // Route was completed and no longer followed
    assertTrue(routeMgr.routeCompletionDeclared());
    routeMgr.resetRouteCompletionStatus();
  }

  /**
   * Tests the detection of a vehicle leaving a route
   * @throws Exception
   */
  @Test
  public void testLeavingRouteVicinity() throws Exception {
    MockRouteManager routeMgr = new MockRouteManager();
    RouteWorker routeWorker = new RouteWorker(routeMgr, log, "src/test/resources/routes/", 3, "earth", "host_vehicle");

    routeWorker.handleSystemAlertMsg(MockRouteManager.buildSystemAlert(SystemAlert.DRIVERS_READY, ""));
    assertTrue(routeWorker.systemOkay == true);

    // Test vehicle has been placed just before start of route
    Location navSatFix = new Location(38.95649, -77.15028, 72.0);
    routeMgr.setEarthToHostTransform(transformFromLocation(navSatFix));
    routeWorker.loop();

    assertTrue(routeWorker.getCurrentState() == WorkerState.ROUTE_SELECTION);

    byte response = routeWorker.setActiveRoute("Colonial Farm Rd. Outbound");
    assertTrue(response == SetActiveRouteResponse.NO_ERROR);
    assertTrue(routeWorker.getCurrentState() == WorkerState.WAITING_TO_START);

    response = routeWorker.startActiveRoute();
    assertTrue(response == StartActiveRouteResponse.NO_ERROR);

    assertTrue(routeWorker.currentSegmentIndex == 0); // start of route

    assertTrue(routeWorker.currentSegment.getUptrackWaypoint().getLocation().almostEqual(
      new Location(38.95649, -77.15028, 72.0), 0.0000001, 0.0000001));
    assertTrue(routeWorker.getCurrentState() == WorkerState.FOLLOWING_ROUTE);

    // Test vehicle has been placed after route file starting point
    navSatFix = new Location(38.9564, -77.15035, 72.0);
    routeMgr.setEarthToHostTransform(transformFromLocation(navSatFix));
    routeWorker.loop();

    assertTrue(routeWorker.currentSegmentIndex == 1); // second segment
    assertTrue(routeWorker.getCurrentState() == WorkerState.FOLLOWING_ROUTE);

    // Test vehicle has been placed inside the garage (off the route)
    // 3 off route nav sat msgs are required to consider vehicle off the route
    navSatFix = new Location(38.95633, -77.15011, 72.0);
    routeMgr.setEarthToHostTransform(transformFromLocation(navSatFix));
    routeWorker.loop();

    assertTrue(routeWorker.currentSegmentIndex == 1); // second segment
    assertTrue(routeWorker.leftRouteVicinity());
    assertTrue(routeWorker.getCurrentState() == WorkerState.FOLLOWING_ROUTE);

    routeWorker.loop();

    assertTrue(routeWorker.currentSegmentIndex == 1); // second segment
    assertTrue(routeWorker.leftRouteVicinity());
    assertTrue(routeWorker.getCurrentState() == WorkerState.FOLLOWING_ROUTE);

    routeWorker.loop();

    assertTrue(routeWorker.currentSegmentIndex == 1); // second segment
    assertTrue(routeWorker.leftRouteVicinity());
    assertTrue(routeWorker.getCurrentState() == WorkerState.ROUTE_SELECTION);
  }

  /**
   * Tests route abort
   * @throws Exception
   */
  @Test
  public void testRouteAbort() throws Exception {
    MockRouteManager routeMgr = new MockRouteManager();
    RouteWorker routeWorker =
      new RouteWorker(routeMgr, log, "src/test/resources/routes/", 3, "earth", "host_vehicle");

    routeWorker.handleSystemAlertMsg(MockRouteManager.buildSystemAlert(SystemAlert.DRIVERS_READY, ""));
    assertTrue(routeWorker.systemOkay == true);

    // Test vehicle has been placed just before start of route
    Location navSatFix = new Location(38.95649, -77.15028, 72.0);
    routeMgr.setEarthToHostTransform(transformFromLocation(navSatFix));
    routeWorker.loop();

    assertTrue(routeWorker.getCurrentState() == WorkerState.ROUTE_SELECTION);

    byte response = routeWorker.setActiveRoute("Colonial Farm Rd. Outbound");
    assertTrue(response == SetActiveRouteResponse.NO_ERROR);
    assertTrue(routeWorker.getCurrentState() == WorkerState.WAITING_TO_START);

    response = routeWorker.startActiveRoute();
    assertTrue(response == StartActiveRouteResponse.NO_ERROR);

    assertTrue(routeWorker.currentSegmentIndex == 0); // start of route

    assertTrue(routeWorker.currentSegment.getUptrackWaypoint().getLocation()
      .almostEqual(new Location(38.95649, -77.15028, 72.0), 0.0000001, 0.0000001));
    assertTrue(routeWorker.getCurrentState() == WorkerState.FOLLOWING_ROUTE);

    // Route has been joined time to abort
    assertTrue(routeWorker.abortActiveRoute() == AbortActiveRouteResponse.NO_ERROR);
    assertTrue(routeWorker.getCurrentState() == WorkerState.ROUTE_SELECTION);

    // With no active route try call again
    assertTrue(routeWorker.abortActiveRoute() == AbortActiveRouteResponse.NO_ACTIVE_ROUTE);
    assertTrue(routeWorker.getCurrentState() == WorkerState.ROUTE_SELECTION);
  }

  /**
   * Helper function to convert a lat lon point to a transform of earth -> host_vehicle
   * 
   * @param navSatFix The location
   * @return the Transform in from ecef to the location
   */
  private Transform transformFromLocation(Location navSatFix) {
    Point3D hostVehicleInECEF = gcc.geodesic2Cartesian(navSatFix, Transform.identity());
    Vector3 trans = new Vector3(hostVehicleInECEF.getX(), hostVehicleInECEF.getY(), hostVehicleInECEF.getZ());
    return new Transform(trans, Quaternion.identity());
  }
}
