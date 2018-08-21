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

package gov.dot.fhwa.saxton.carma.signal_plugin;

import gov.dot.fhwa.saxton.carma.geometry.GeodesicCartesianConverter;
import gov.dot.fhwa.saxton.carma.geometry.cartesian.Point3D;
import gov.dot.fhwa.saxton.carma.geometry.cartesian.Vector;
import gov.dot.fhwa.saxton.carma.geometry.cartesian.Vector3D;
import gov.dot.fhwa.saxton.carma.geometry.geodesic.Location;
import gov.dot.fhwa.saxton.carma.guidance.util.trajectoryconverter.RoutePointStamped;
import gov.dot.fhwa.saxton.carma.signal_plugin.ead.PlanInterpolator;
import gov.dot.fhwa.saxton.carma.signal_plugin.ead.SimpleNCVMotionPredictor;

import java.util.Arrays;
import java.util.List;
import java.util.ArrayList;
import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import org.ros.message.MessageFactory;
import org.ros.node.NodeConfiguration;
import org.ros.rosjava_geometry.Transform;

import sensor_msgs.NavSatFix;
import sensor_msgs.NavSatStatus;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
import gov.dot.fhwa.saxton.carma.signal_plugin.ead.trajectorytree.Node;

/**
 * Runs unit tests for the RouteWorker class
 */
public class SimpleNCVMotionPredictorTest {

  Log log;

  @Before
  public void setUp() throws Exception {
    log = LogFactory.getLog(SimpleNCVMotionPredictor.class);
    log.info("Setting up tests for SimpleNCVMotionPredictor");
  }

  @After
  public void tearDown() throws Exception {
  }

  /**
   * Tests the interpolation of vehicle position between two nodes
   * @throws Exception
   */
  @Test
  public void testPredictMotion() throws Exception {
    SimpleNCVMotionPredictor motionPredictor = new SimpleNCVMotionPredictor();


    // Empty list will return empty list
    List<RoutePointStamped> points = motionPredictor.predictMotion("1", new ArrayList<Node>(), 0.2, 1.0);
    assertTrue(points.isEmpty());

    // A single node will return an empty list
    Node n1 = new Node(10.0, 0.0, 5.0);
    points = motionPredictor.predictMotion("1", Arrays.asList(n1), 0.2, 1.0);
    assertTrue(points.isEmpty());

    // 2 Node list with 0 acceleration
    n1 = new Node(10.0, 0.0, 5.0);
    Node n2 = new Node(15.0, 1.0, 5.0);
    points = motionPredictor.predictMotion("1", Arrays.asList(n1,n2), 0.2, 1.0);

    assertEquals(5, points.size());
    assertTrue(routePointAlmostEqual(new RoutePointStamped(16.0, 0.0, 1.2), points.get(0), 0.0001));
    assertTrue(routePointAlmostEqual(new RoutePointStamped(17.0, 0.0, 1.4), points.get(1), 0.0001));
    assertTrue(routePointAlmostEqual(new RoutePointStamped(18.0, 0.0, 1.6), points.get(2), 0.0001));
    assertTrue(routePointAlmostEqual(new RoutePointStamped(19.0, 0.0, 1.8), points.get(3), 0.0001));
    assertTrue(routePointAlmostEqual(new RoutePointStamped(20.0, 0.0, 2.0), points.get(4), 0.0001));

    // TODO // 5 Node list with complex motion
    // n1 = new Node(10.0, 0.0, 5.0);
    // n2 = new Node(40.0, 4.0, 10.0);
    // Node n3 = new Node(70.0, 8.0, 10.0);
    // Node n4 = new Node(70.0, 12.0, 5.0);
    // Node n5 = new Node(70.0, 8.0, 5.0); // Finish building list
    // points = planInterpolator.interpolateMotion(Arrays.asList(n1,n2), 1.0);

    // assertTrue(routePointAlmostEqual(new RoutePointStamped(10.0, 0.0, 0.0), points.get(0), 0.0)); // Inclusive first node
    // assertTrue(routePointAlmostEqual(new RoutePointStamped(40.0, 0.0, 4.0), points.get(points.size() - 1), 0.0001)); // Include last node
    // assertEquals(points.size(), 5);
    // assertTrue(routePointAlmostEqual(new RoutePointStamped(15.625, 0.0, 1.0), points.get(1), 0.0001));
    // assertTrue(routePointAlmostEqual(new RoutePointStamped(22.5, 0.0, 2.0), points.get(2), 0.0001));
    // assertTrue(routePointAlmostEqual(new RoutePointStamped(30.625, 0.0, 3.0), points.get(3), 0.0001));
  }

  /**
   * Helper function to comparing two RoutePointStamped objects for equality
   */
  boolean routePointAlmostEqual(RoutePointStamped r1, RoutePointStamped r2, double epsilon) {
    boolean result = r1.getPoint().almostEquals(r2.getPoint(), epsilon) 
      && r1.getSegmentIdx() == r2.getSegmentIdx() 
      && Math.abs(r1.getSegDowntrack() - r2.getSegDowntrack()) <= epsilon;

    if (!result) {
      log.info(r1  + " and " + r2 + " are not equal within " + epsilon);
    }
    return result;
  }
}
