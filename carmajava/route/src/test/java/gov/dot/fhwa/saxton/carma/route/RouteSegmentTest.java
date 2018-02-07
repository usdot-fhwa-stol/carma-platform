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
import org.ros.rosjava_geometry.Quaternion;
import org.ros.rosjava_geometry.Transform;
import org.ros.rosjava_geometry.Vector3;
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
   * Tests the determination of the segment frame
   * Note: This test is for a route following the center line of the direction of travel
   * @throws Exception
   */
  @Test
  public void testGetSegmentAllignedFRDFrame() throws Exception {
    // Test segment going east
    Location loc1 = new Location(0,0,0);
    Location loc2 = new Location(0, 0.00001, 0); // Approximately 1m long segment
    RouteWaypoint wpUp = new RouteWaypoint(loc1);
    RouteWaypoint wpDown = new RouteWaypoint(loc2);
    RouteSegment seg = new RouteSegment(wpUp, wpDown);
    Transform ecefToSeg = seg.getECEFToSegmentTransform();
    
    Vector3 solutionTrans = new Vector3(6378137.0, 0, 0);
    Quaternion solutionRot = new Quaternion(-0.5, -0.5, 0.5, 0.5);// +90 around Z and -90 around new X
    
    assertTrue(ecefToSeg.getTranslation().almostEquals(solutionTrans, 0.1));// Check accuracy to within 0.1m
    assertTrue(ecefToSeg.getRotationAndScale().almostEquals(solutionRot, 0.0001)); // Check accuracy to within ~0.01 deg

    // Test segment going west
    loc1 = new Location(0,0,0);
    loc2 = new Location(0, -0.00001, 0); // Approximately 1m long segment
    wpUp = new RouteWaypoint(loc1);
    wpDown = new RouteWaypoint(loc2);
    seg = new RouteSegment(wpUp, wpDown);
    ecefToSeg = seg.getECEFToSegmentTransform();

    solutionTrans = new Vector3(6378137.0, 0, 0);
    solutionRot = new Quaternion(-0.5, 0.5, 0.5, -0.5);// -90 around Z and +90 around new X
    
    assertTrue(ecefToSeg.getTranslation().almostEquals(solutionTrans, 0.1));// Check accuracy to within 0.1m
    assertTrue(ecefToSeg.getRotationAndScale().almostEquals(solutionRot, 0.0001)); // Check accuracy to within ~0.01 deg

    // Test segment going north
    loc1 = new Location(0,0,0);
    loc2 = new Location(0.00001, 0.0, 0); // Approximately 1m long segment
    wpUp = new RouteWaypoint(loc1);
    wpDown = new RouteWaypoint(loc2);
    seg = new RouteSegment(wpUp, wpDown);
    ecefToSeg = seg.getECEFToSegmentTransform();
    
    solutionTrans = new Vector3(6378137.0, 0, 0);
    solutionRot = new Quaternion(0.0, -0.7071068, 0.0, 0.7071068);// -90 around new Y
    
    assertTrue(ecefToSeg.getTranslation().almostEquals(solutionTrans, 0.1));// Check accuracy to within 0.1m
    assertTrue(ecefToSeg.getRotationAndScale().almostEquals(solutionRot, 0.0001)); // Check accuracy to within ~0.01 deg

    // Test segment going south
    loc1 = new Location(0,0,0);
    loc2 = new Location(-0.00001, 0.0, 0); // Approximately 1m long segment
    wpUp = new RouteWaypoint(loc1);
    wpDown = new RouteWaypoint(loc2);
    seg = new RouteSegment(wpUp, wpDown);
    ecefToSeg = seg.getECEFToSegmentTransform();
    
    solutionTrans = new Vector3(6378137.0, 0, 0);
    solutionRot = new Quaternion(-0.7071068, 0.0, 0.7071068, 0.0);//+90 around Y and 180 around new X
    
    assertTrue(ecefToSeg.getTranslation().almostEquals(solutionTrans, 0.1));// Check accuracy to within 0.1m
    assertTrue(ecefToSeg.getRotationAndScale().almostEquals(solutionRot, 0.0001)); // Check accuracy to within ~0.01 deg
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