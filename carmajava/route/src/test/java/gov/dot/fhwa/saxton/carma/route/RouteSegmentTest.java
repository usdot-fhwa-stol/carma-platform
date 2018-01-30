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

package gov.dot.fhwa.saxton.carma.route;

import cav_msgs.SystemAlert;
import cav_srvs.AbortActiveRouteResponse;
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
import sensor_msgs.NavSatFix;
import sensor_msgs.NavSatStatus;

import static org.junit.Assert.assertTrue;
import static org.junit.Assert.assertEquals;

/**
 * Runs unit tests for the RouteSegment class
 */
public class RouteSegmentTest {

  Log log;
  NodeConfiguration nodeConfiguration = NodeConfiguration.newPrivate();
  MessageFactory messageFactory = nodeConfiguration.getTopicMessageFactory();

  @Before
  public void setUp() throws Exception {
    log = LogFactory.getLog(RouteSegmentTest.class);
    log.info("Setting up tests for Route Segment");
  }

  @After
  public void tearDown() throws Exception {
  }

  /**
   * Tests the movement of a host vehicle along a route
   * Note: This test is for a route following the center line of the direction of travel
   * @throws Exception
   */
  @Test
  public void testDeterminePrimaryLane() throws Exception {
    Location loc1 = new Location(38.95511,-77.15171, 0);
    Location loc2 = new Location(38.95551, -77.15147, 0);
    RouteWaypoint wpUp = new RouteWaypoint(loc1);
    RouteWaypoint wpDown = new RouteWaypoint(loc2);
    RouteSegment seg = new RouteSegment(wpUp, wpDown);

    wpDown.setLaneWidth(4.0);
    // Test 1 lane
    wpDown.setLaneCount(1);

    int result = seg.determinePrimaryLane(2.0); // 2m crosstrack
    assertEquals(0, result);

    result = seg.determinePrimaryLane(-2.0); // -2m crosstrack
    assertEquals(0, result);

    result = seg.determinePrimaryLane(-5.0); // -5m crosstrack projected onto fake lanes
    assertEquals(1, result);

    result = seg.determinePrimaryLane(5.0); // -5m crosstrack projected onto fake lanes
    assertEquals(-1, result);

    result = seg.determinePrimaryLane(0.0); // 0m crosstrack
    assertEquals(0, result);

    // Test 2 lane
    wpDown.setLaneCount(2);

    result = seg.determinePrimaryLane(2.0); // 2m crosstrack
    assertEquals(0, result);

    result = seg.determinePrimaryLane(-2.0); // -2m crosstrack
    assertEquals(1, result);

    result = seg.determinePrimaryLane(-5.0); // -5m crosstrack projected onto fake lanes
    assertEquals(2, result);

    result = seg.determinePrimaryLane(5.0); // -5m crosstrack projected onto fake lanes
    assertEquals(-1, result);

    result = seg.determinePrimaryLane(0.0); // 0m crosstrack
    assertEquals(0, result);

    // Test 3 lane
    wpDown.setLaneCount(3);

    result = seg.determinePrimaryLane(2.0); // 2m crosstrack
    assertEquals(1, result);

    result = seg.determinePrimaryLane(-2.0); // -2m crosstrack
    assertEquals(1, result);

    result = seg.determinePrimaryLane(-5.0); // -5m crosstrack
    assertEquals(2, result);

    result = seg.determinePrimaryLane(5.0); // -5m crosstrack
    assertEquals(0, result);

    result = seg.determinePrimaryLane(-9.0); // -5m crosstrack
    assertEquals(3, result);

    result = seg.determinePrimaryLane(9.0); // -5m crosstrack
    assertEquals(-1, result);

    result = seg.determinePrimaryLane(0.0); // 0m crosstrack
    assertEquals(1, result);

    // Test 4 lane
    wpDown.setLaneCount(4);

    result = seg.determinePrimaryLane(2.0); // 2m crosstrack
    assertEquals(1, result);

    result = seg.determinePrimaryLane(-2.0); // -2m crosstrack
    assertEquals(2, result);

    result = seg.determinePrimaryLane(-5.0); // -5m crosstrack
    assertEquals(3, result);

    result = seg.determinePrimaryLane(5.0); // -5m crosstrack
    assertEquals(0, result);

    result = seg.determinePrimaryLane(-9.0); // -5m crosstrack
    assertEquals(4, result);

    result = seg.determinePrimaryLane(9.0); // -5m crosstrack
    assertEquals(-1, result);

    result = seg.determinePrimaryLane(0.0); // 0m crosstrack
    assertEquals(1, result);
    
  }
}