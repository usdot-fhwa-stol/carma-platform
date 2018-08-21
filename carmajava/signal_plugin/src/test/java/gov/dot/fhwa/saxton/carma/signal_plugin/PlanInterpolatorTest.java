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

/**
 * Runs unit tests for the RouteWorker class
 */
public class PlanInterpolatorTest {

  Log log;
  NodeConfiguration nodeConfiguration = NodeConfiguration.newPrivate();
  MessageFactory messageFactory = nodeConfiguration.getTopicMessageFactory();

  @Before
  public void setUp() throws Exception {
    log = LogFactory.getLog(RouteTest.class);
    log.info("Setting up tests for RouteFunctions");
  }

  @After
  public void tearDown() throws Exception {
  }

  /**
   * Tests the movement of a host vehicle along a route
   * @throws Exception
   */
  @Test
  public void testInterpolateMotion() throws Exception {
    PlanInterpolator planInterpolator = new PlanInterpolator();


    // Single node should return empty list TODO Should it return the one node?
    List<RoutePointStamped> points = planInterpolator.interpolateMotion(trajectory, timeStep);
    assertTrue(points.isEmpty());

    // TODO List of two identical nodes Should it return the one point?

    // 2 Node list with 0 acceleration
    Node n1 = new Node(10.0, 0.0, 5.0);
    Node n2 = new Node(15.0, 1.0, 5.0);
    points = planInterpolator.interpolateMotion(Arrays.asList(n1,n2), 0.2);

    assertEquals(new RoutePointStamped(10.0, 0.0, 0.0), points.get(0)); // Inclusive first node
    assertEquals(new RoutePointStamped(15.0, 2.0, 0.0), points.get(points.size() - 1)); // Include last node
    assertEquals(points.size(), 6);
    assertEquals(new RoutePointStamped(11.0, 0.0, 0.0), points.get(1));
    assertEquals(new RoutePointStamped(12.0, 0.0, 0.0), points.get(2));
    assertEquals(new RoutePointStamped(13.0, 0.0, 0.0), points.get(3));
    assertEquals(new RoutePointStamped(14.0, 0.0, 0.0), points.get(4));
  }
}
