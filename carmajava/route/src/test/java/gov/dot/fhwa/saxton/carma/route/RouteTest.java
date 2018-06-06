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
import gov.dot.fhwa.saxton.carma.geometry.GeodesicCartesianConverter;
import gov.dot.fhwa.saxton.carma.geometry.cartesian.Point3D;
import gov.dot.fhwa.saxton.carma.geometry.cartesian.Vector;
import gov.dot.fhwa.saxton.carma.geometry.cartesian.Vector3D;
import gov.dot.fhwa.saxton.carma.geometry.geodesic.Location;
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
public class RouteTest {

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
  public void testFindRouteSubsection() throws Exception {
    FileStrategy fS = new FileStrategy("src/test/resources/routes/colonial_farm_rd_outbound.yaml", log);
    Route route = Route.fromMessage(fS.load().toMessage(messageFactory)); // Load route with waypoint ids assigned.
    assertTrue(route != null);

    // Test vehicle has been placed at segment 6 17m downtrack of segment
    // start segment 6
    // 170 m front = segment 9
    // 120 m back = segment 3
    int startingIndex = 6;
    double downtrackAtSegment6 = 17.0;
    double distBackward = 120.0;
    double distForward = 170.0;
    List<RouteSegment> subRoute = route.findRouteSubsection(startingIndex, downtrackAtSegment6, distBackward, distForward);
    int solutionMinIndex = 3;
    int solutionMaxIndex = 9;
    for (int i=solutionMinIndex; i<=solutionMaxIndex; i++) {
      assertEquals(i, subRoute.get(i - solutionMinIndex).getUptrackWaypoint().getWaypointId());
    }
  }

      /**
   * Tests the movement of a host vehicle along a route
   * @throws Exception
   */
  @Test
  public void testGetLength() throws Exception {
    FileStrategy fS = new FileStrategy("src/test/resources/routes/SB_Platoon_65mph.yaml", log);
    Route route = Route.fromMessage(fS.load().toMessage(messageFactory)); // Load route with waypoint ids assigned.

    double lengthStartToMeter = route.lengthOfSegments(0, 18); // NB_M 20th wp, SB_M NOTE: These WP are not all correct. I measured with Gmaps instead
    double lengthStartToMerge = route.lengthOfSegments(0, 480); // NB_M 30, NB 711th WP, SB_M, SB 482nd WP
    double mergeToEndMerge = route.lengthOfSegments(30, 35); // NB_M end 36, SB_M end 
    double meterToMerge = route.lengthOfSegments(18, 30);
    System.out.println("\n\n");
    System.out.println(route.getSegments().get(29).getDowntrackWaypoint().getLocation());
    System.out.println("start to meter: " + lengthStartToMeter);
    System.out.println("start to merge: " + lengthStartToMerge);
    System.out.println(" length of merge: " + mergeToEndMerge);
    System.out.println(" meter to merge: " + meterToMerge);
    System.out.println("Route length: " + route.getRouteLength());
    System.out.println("\n\n");
  }

  /**
   * Calculates the extra point to make two routes have equal length
   * Assumes route assignments are correct (long vs short)
   * Assumes that the routes intersect at a common point and then continue along the same geometry to the end
   * Use for TO26 Route updates
   * @throws Exception
   */
  @Test
  public void testSetEqual() throws Exception {
    System.out.println("\n\n");
    FileStrategy fS = new FileStrategy("src/test/resources/routes/NB_merge.yaml", log);
    Route shortRoute = Route.fromMessage(fS.load().toMessage(messageFactory)); // Load route with waypoint ids assigned.
    double shortDist = shortRoute.getRouteLength();

    fS = new FileStrategy("src/test/resources/routes/NB_Platoon_65MPH.yaml", log);
    Route longRoute = Route.fromMessage(fS.load().toMessage(messageFactory)); // Load route with waypoint ids assigned.
    double longDist = longRoute.getRouteLength();
    System.out.println("Short Route length: " + shortDist);
    System.out.println("Long Route length: " + longDist);
    System.out.println("ShortRoute Elev: " + shortRoute.getFirstSegment().getUptrackWaypoint().getLocation().getAltitude());
    double diff = longDist - shortDist;
    System.out.println("Diff: " + diff);

    Point3D firstWP = shortRoute.getFirstSegment().getUptrackWaypoint().getECEFPoint();
    Point3D secondWP = shortRoute.getFirstSegment().getDowntrackWaypoint().getECEFPoint();

    Vector unitVec = new Vector3D(secondWP, firstWP).getUnitVector();
    Vector3D newStart = Vector3D.fromVector(new Vector3D(firstWP).add(unitVec.scalarMultiply(diff)));

    Point3D newStartPoint = new Point3D(newStart.getX(),newStart.getY(),newStart.getZ());
    GeodesicCartesianConverter gcc = new GeodesicCartesianConverter();
    Location newWP = gcc.cartesian2Geodesic(newStartPoint, Transform.identity());
    shortRoute.insertWaypoint(new RouteWaypoint(newWP), 0);
    System.out.println("New " + newWP);
    System.out.println("New Length: " + shortRoute.getRouteLength());
    System.out.println("\n\n");
  }

}
