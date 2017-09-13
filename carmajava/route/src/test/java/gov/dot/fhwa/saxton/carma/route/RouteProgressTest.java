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

package gov.dot.fhwa.saxton.carma.route;

import cav_msgs.RouteState;
import cav_srvs.SetActiveRouteRequest;
import cav_srvs.SetActiveRouteResponse;
import gov.dot.fhwa.saxton.carma.geometry.geodesic.GreatCircleSegment;
import gov.dot.fhwa.saxton.carma.geometry.geodesic.HaversineStrategy;
import gov.dot.fhwa.saxton.carma.geometry.geodesic.Location;
import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import org.ros.message.MessageFactory;
import org.ros.node.NodeConfiguration;
import sensor_msgs.NavSatFix;
import sensor_msgs.NavSatStatus;

import static org.junit.Assert.assertTrue;

/**
 * Runs unit tests for the GeodesicCartesianConvertor class
 */
public class RouteProgressTest {

  Log log;
  NodeConfiguration nodeConfiguration = NodeConfiguration.newPrivate();
  MessageFactory messageFactory = nodeConfiguration.getTopicMessageFactory();

  @Before
  public void setUp() throws Exception {
    log = LogFactory.getLog(RouteProgressTest.class);
    log.info("Setting up tests for RouteProgressFunctions");
  }

  @After
  public void tearDown() throws Exception {
  }

  /**
   * Tests the distanceLoc2Loc function
   * Accuracy checked against online calculator http://www.movable-type.co.uk/scripts/latlong.html
   * @throws Exception
   */
  @Test
  public void testRouteProgress() throws Exception {
    RouteWorker routeWorker = new RouteWorker(new FakeRouteManager(), log,
      "/home/mcconnelms/to13_ws/src/CarmaPlatform/carmajava/route/src/test/resources/routefiles/");

    // Test vehicle has been placed just before start of route
    NavSatFix navMsg = messageFactory.newFromType(NavSatFix._TYPE);
    navMsg.getStatus().setStatus(NavSatStatus.STATUS_FIX);
    navMsg.setLatitude(38.95649);
    navMsg.setLongitude(-77.15028);
    navMsg.setAltitude(0);
    routeWorker.handleNavSatFixMsg(navMsg);

    assertTrue(routeWorker.getState() == WorkerState.ROUTE_SELECTION);

    SetActiveRouteRequest request = messageFactory.newFromType(SetActiveRouteRequest._TYPE);
    request.setRouteID("Colonial Farm Rd. Outbound");
    SetActiveRouteResponse response = routeWorker.setActiveRoute(request);
    assertTrue(response.getErrorStatus() != SetActiveRouteResponse.NO_ERROR);
    assertTrue(routeWorker.getState() == WorkerState.FOLLOWING_ROUTE);

    assertTrue(routeWorker.currentSegmentIndex == 0); // start of route

    // No change in location
    routeWorker.handleNavSatFixMsg(navMsg);
    assertTrue(routeWorker.currentSegmentIndex == 0); // start of route
    assertTrue(routeWorker.getState() == WorkerState.FOLLOWING_ROUTE);

    // Test vehicle has been placed after route file starting point
    navMsg.getStatus().setStatus(NavSatStatus.STATUS_FIX);
    navMsg.setLatitude(38.9564);
    navMsg.setLongitude(-77.15035);
    navMsg.setAltitude(0);
    routeWorker.handleNavSatFixMsg(navMsg);

    assertTrue(routeWorker.currentSegmentIndex == 1); // second segment
    assertTrue(routeWorker.getState() == WorkerState.FOLLOWING_ROUTE);

    // Test vehicle has been placed after next point
    navMsg.getStatus().setStatus(NavSatStatus.STATUS_FIX);
    navMsg.setLatitude(38.95631);
    navMsg.setLongitude(-77.15042);
    navMsg.setAltitude(0);
    routeWorker.handleNavSatFixMsg(navMsg);

    assertTrue(routeWorker.currentSegmentIndex == 2); // third segment
    assertTrue(routeWorker.getState() == WorkerState.FOLLOWING_ROUTE);

    // Test vehicle has been placed after end of route
    navMsg.getStatus().setStatus(NavSatStatus.STATUS_FIX);
    navMsg.setLatitude(38.95218);
    navMsg.setLongitude(-77.15189);
    navMsg.setAltitude(0);
    routeWorker.handleNavSatFixMsg(navMsg);

    assertTrue(routeWorker.currentSegmentIndex == 2); // third segment
    assertTrue(routeWorker.getState() == WorkerState.);

    assertTrue(!routeWorker.atNextSegment()); // no change in location so vehicle should not be at the next segment
    assertTrue(!routeWorker.leftRouteVicinity()); // no movement so still on route
    assertTrue(!routeWorker.routeCompleted()); // no movement so route not completed

    routeWorker.
//    downtrackDistance = currentSegment.downTrackDistance(hostVehicleLocation);
//    crossTrackDistance = currentSegment.crossTrackDistance(hostVehicleLocation);
//
//    if (atNextSegment()) {
//      currentSegmentIndex++;
//      currentSegment = activeRoute.getSegments().get(currentSegmentIndex);
//    }
//
//    if (routeCompleted()) {
//      handleStateTransition(WorkerEvent.ROUTE_COMPLETED);
//    }
//
//    if (leftRouteVicinity()) {
//      handleStateTransition(WorkerEvent.LEFT_ROUTE);
//    }
//    log.info("// Entering distanceLoc2Loc test");
//    HaversineStrategy haversineStrategy = new HaversineStrategy();
//
//    // Test points on example garage to colonial farm rd route
//    Location loc1 = new Location(38.95647,-77.15031, 0);
//    Location loc2 = new Location(38.95631, -77.15041, 0);
//    double solution = 19.78;
//    assertTrue(Math.abs(haversineStrategy.distanceLoc2Loc(loc1, loc2) - solution) < 0.1); // Check accuracy to within .1m
//
//    loc1 = new Location(38.95628,-77.15047, 0);
//    loc2 = new Location(38.95613, -77.15101, 0);
//    solution = 49.58;
//    assertTrue(Math.abs(haversineStrategy.distanceLoc2Loc(loc1, loc2) - solution) < 0.1); // Check accuracy to within .1m
//
//    // Test points about a km apart
//    loc1 = new Location(38.942201,-77.160108, 0);
//    loc2 = new Location(38.943804, -77.148832, 0);
//    solution = 991.4;
//    assertTrue(Math.abs(haversineStrategy.distanceLoc2Loc(loc1, loc2) - solution) < 0.1); // Check accuracy to within .1m
  }
}
