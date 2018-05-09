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
import java.util.List;
import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import org.ros.message.MessageFactory;
import org.ros.node.NodeConfiguration;
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
    FileStrategy fS = new FileStrategy("src/test/resources/routes/Merge.yaml", log);
    Route route = Route.fromMessage(fS.load().toMessage(messageFactory)); // Load route with waypoint ids assigned.

    double lengthStartToMeter = route.lengthOfSegments(0, 130 - 95);
    double lengthStartToMerge = route.lengthOfSegments(0, 140 - 95);
    double mergeToEndMerge = route.lengthOfSegments(140 - 95, 152 - 95);
    double meterToMerge = route.lengthOfSegments(130 - 95, 140- 95);
    System.out.println("\n\n");
    System.out.println("start to meter: " + lengthStartToMeter);
    System.out.println("start to merge: " + lengthStartToMerge);
    System.out.println(" length of merge: " + mergeToEndMerge);
    System.out.println(" meter to merge: " + meterToMerge);
    System.out.println("\n\n");
  }
}
