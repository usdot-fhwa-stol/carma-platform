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

package gov.dot.fhwa.saxton.carma.roadway;

import cav_msgs.SystemAlert;
import cav_srvs.SetActiveRouteResponse;
import cav_srvs.StartActiveRouteResponse;
import gov.dot.fhwa.saxton.carma.geometry.geodesic.Location;
import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import org.ros.message.MessageFactory;
import org.ros.node.NodeConfiguration;
import org.ros.rosjava_geometry.Quaternion;
import sensor_msgs.NavSatFix;
import sensor_msgs.NavSatStatus;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/**
 * Runs unit tests for the RouteWorker class
 */
public class RoadwayWorkerTest {

  Log log;
  NodeConfiguration nodeConfiguration = NodeConfiguration.newPrivate();
  MessageFactory messageFactory = nodeConfiguration.getTopicMessageFactory();

  @Before
  public void setUp() throws Exception {
    log = LogFactory.getLog(RoadwayWorkerTest.class);
    log.info("Setting up tests for RoadwayWorker");
  }

  @After
  public void tearDown() throws Exception {
  }

  /**
   * Tests the detection of a vehicle leaving a route
   * @throws Exception
   */
  @Test
  public void testLeavingRouteVicinity() throws Exception {

  }


//  /**
//   * Tests the detection of a vehicle leaving a route
//   * @throws Exception
//   */
//  @Test
//  public void testLeavingRouteVicinity() throws Exception {
//    RouteWorker routeWorker = new RouteWorker(new MockRouteManager(), log, "src/test/resources/routefiles/");
//
//    routeWorker.handleSystemAlertMsg(routeWorker.buildSystemAlertMsg(SystemAlert.DRIVERS_READY, ""));
//    assertTrue(routeWorker.systemOkay == true);
//
//    // Test vehicle has been placed just before start of route
//    NavSatFix navMsg = messageFactory.newFromType(NavSatFix._TYPE);
//    navMsg.getStatus().setStatus(NavSatStatus.STATUS_FIX);
//    navMsg.setLatitude(38.95649);
//    navMsg.setLongitude(-77.15028);
//    navMsg.setAltitude(0);
//    routeWorker.handleNavSatFixMsg(navMsg);
//
//    assertTrue(routeWorker.getCurrentState() == WorkerState.ROUTE_SELECTION);
//
//    byte response = routeWorker.setActiveRoute("Colonial Farm Rd. Outbound");
//    assertTrue(response == SetActiveRouteResponse.NO_ERROR);
//    assertTrue(routeWorker.getCurrentState() == WorkerState.WAITING_TO_START);
//
//    response = routeWorker.startActiveRoute();
//    assertTrue(response == StartActiveRouteResponse.NO_ERROR);
//
//    assertTrue(routeWorker.currentSegmentIndex == 0); // start of route
//
//    assertTrue(routeWorker.currentSegment.getUptrackWaypoint().getLocation().almostEqual(
//      new Location(38.95649, -77.15028, 0), 0.0000001, 0.0000001));
//    assertTrue(routeWorker.getCurrentState() == WorkerState.FOLLOWING_ROUTE);
//
//    // Test vehicle has been placed after route file starting point
//    navMsg.getStatus().setStatus(NavSatStatus.STATUS_FIX);
//    navMsg.setLatitude(38.9564);
//    navMsg.setLongitude(-77.15035);
//    navMsg.setAltitude(0);
//    routeWorker.handleNavSatFixMsg(navMsg);
//
//    assertTrue(routeWorker.currentSegmentIndex == 1); // second segment
//    assertTrue(routeWorker.getCurrentState() == WorkerState.FOLLOWING_ROUTE);
//
//    // Test vehicle has been placed inside the garage (off the route)
//    navMsg.getStatus().setStatus(NavSatStatus.STATUS_FIX);
//    navMsg.setLatitude(38.95633);
//    navMsg.setLongitude(-77.15011);
//    navMsg.setAltitude(0);
//    routeWorker.handleNavSatFixMsg(navMsg);
//
//    assertTrue(routeWorker.currentSegmentIndex == 1); // second segment
//    assertTrue(routeWorker.leftRouteVicinity());
//    assertTrue(routeWorker.getCurrentState() == WorkerState.WAITING_TO_START);
//  }
}
