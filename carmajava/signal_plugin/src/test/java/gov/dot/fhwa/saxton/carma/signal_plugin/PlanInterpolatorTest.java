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
 * Runs unit tests for the PlanInterpolator class
 */
public class PlanInterpolatorTest {

  Log log;

  @Before
  public void setUp() throws Exception {
    log = LogFactory.getLog(PlanInterpolatorTest.class);
    log.info("Setting up tests for PlanInterpolator");
  }

  @After
  public void tearDown() throws Exception {
  }

  /**
   * Tests the interpolation of vehicle position between two nodes
   * @throws Exception
   */
  @Test
  public void testInterpolateMotion() throws Exception {
    PlanInterpolator planInterpolator = new PlanInterpolator();


    // Empty list will return empty list
    List<RoutePointStamped> points = planInterpolator.interpolateMotion(new ArrayList<Node>(), 0.2, 0, 0);
    assertTrue(points.isEmpty());

    // A single node will return a single point
    Node n1 = new Node(10.0, 0.0, 5.0);
    points = planInterpolator.interpolateMotion(Arrays.asList(n1), 0.2, 0, 0);
    assertEquals(points.size(), 1);
    assertTrue(routePointAlmostEqual(new RoutePointStamped(10.0, 0.0, 0.0), points.get(0), 0.0));

    // 2 Node list with 0 acceleration
    n1 = new Node(10.0, 0.0, 5.0);
    Node n2 = new Node(15.0, 1.0, 5.0);
    points = planInterpolator.interpolateMotion(Arrays.asList(n1,n2), 1.0, 0, 0);

    assertTrue(routePointAlmostEqual(new RoutePointStamped(10.0, 0.0, 0.0), points.get(0), 0.0)); // Inclusive first node
    assertTrue(routePointAlmostEqual(new RoutePointStamped(15.0, 0.0, 1.0), points.get(points.size() - 1), 0.0001)); // Include last node
    assertEquals(points.size(), 6);
    assertTrue(routePointAlmostEqual(new RoutePointStamped(11.0, 0.0, 0.2), points.get(1), 0.0001));
    assertTrue(routePointAlmostEqual(new RoutePointStamped(12.0, 0.0, 0.4), points.get(2), 0.0001));
    assertTrue(routePointAlmostEqual(new RoutePointStamped(13.0, 0.0, 0.6), points.get(3), 0.0001));
    assertTrue(routePointAlmostEqual(new RoutePointStamped(14.0, 0.0, 0.8), points.get(4), 0.0001));

    // 2 Node list with positive acceleration
    n1 = new Node(10.0, 0.0, 5.0);
    n2 = new Node(40.0, 4.0, 10.0);
    points = planInterpolator.interpolateMotion(Arrays.asList(n1,n2), 5.0, 0, 0);

    assertTrue(routePointAlmostEqual(new RoutePointStamped(10.0, 0.0, 0.0), points.get(0), 0.0)); // Inclusive first node
    assertTrue(routePointAlmostEqual(new RoutePointStamped(40.0, 0.0, 4.0), points.get(points.size() - 1), 0.0001)); // Include last node
    assertEquals(7, points.size());
    assertTrue(routePointAlmostEqual(new RoutePointStamped(15, 0.0, 0.89897948556), points.get(1), 0.0001));
    assertTrue(routePointAlmostEqual(new RoutePointStamped(20, 0.0, 1.65685424949), points.get(2), 0.0001));
    assertTrue(routePointAlmostEqual(new RoutePointStamped(25, 0.0, 2.32455532034), points.get(3), 0.0001));
    assertTrue(routePointAlmostEqual(new RoutePointStamped(30, 0.0, 2.92820323028), points.get(4), 0.0001));
    assertTrue(routePointAlmostEqual(new RoutePointStamped(35, 0.0, 3.48331477355), points.get(5), 0.0001));

    // 3 Node list with acceleration flip
    n1 = new Node(10.0, 0.0, 5.0);
    n2 = new Node(40.0, 4.0, 10.0);
    Node n3 = new Node(70.0, 8.0, 5.0);
    points = planInterpolator.interpolateMotion(Arrays.asList(n1,n2, n3), 5.0, 0, 0);
    assertTrue(routePointAlmostEqual(new RoutePointStamped(10.0, 0.0, 0.0), points.get(0), 0.0)); // Inclusive first node
    assertTrue(routePointAlmostEqual(new RoutePointStamped(70.0, 0.0, 8.0), points.get(points.size() - 1), 0.0001)); // Include last node
    assertEquals(13, points.size());
    assertTrue(routePointAlmostEqual(new RoutePointStamped(15, 0.0, 0.89897948556), points.get(1), 0.0001));
    assertTrue(routePointAlmostEqual(new RoutePointStamped(20, 0.0, 1.65685424949), points.get(2), 0.0001));
    assertTrue(routePointAlmostEqual(new RoutePointStamped(25, 0.0, 2.32455532034), points.get(3), 0.0001));
    assertTrue(routePointAlmostEqual(new RoutePointStamped(30, 0.0, 2.92820323028), points.get(4), 0.0001));
    assertTrue(routePointAlmostEqual(new RoutePointStamped(35, 0.0, 3.48331477355), points.get(5), 0.0001));
    assertTrue(routePointAlmostEqual(new RoutePointStamped(40, 0.0, 4.0), points.get(6), 0.0001));
    assertTrue(routePointAlmostEqual(new RoutePointStamped(45, 0.0, 4.51668522645), points.get(7), 0.0001));
    assertTrue(routePointAlmostEqual(new RoutePointStamped(50, 0.0, 5.07179676972), points.get(8), 0.0001));
    assertTrue(routePointAlmostEqual(new RoutePointStamped(55, 0.0, 5.67544467966), points.get(9), 0.0001));
    assertTrue(routePointAlmostEqual(new RoutePointStamped(60, 0.0, 6.34314575051), points.get(10), 0.0001));
    assertTrue(routePointAlmostEqual(new RoutePointStamped(65, 0.0, 7.10102051443), points.get(11), 0.0001));

    // 2 Node list with possible overshoot avoided
    n1 = new Node(10.0, 0.0, 5.0);
    n2 = new Node(15.0, 1.0, 5.0);
    points = planInterpolator.interpolateMotion(Arrays.asList(n1,n2), 2.75, 0, 0);

    assertTrue(routePointAlmostEqual(new RoutePointStamped(10.0, 0.0, 0.0), points.get(0), 0.0)); // Inclusive first node
    assertTrue(routePointAlmostEqual(new RoutePointStamped(15.0, 0.0, 1.0), points.get(points.size() - 1), 0.0001)); // Include last node
    assertEquals(3, points.size());
    assertTrue(routePointAlmostEqual(new RoutePointStamped(12.75, 0.0, 0.55), points.get(1), 0.0001));
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
