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

package gov.dot.fhwa.saxton.carma.signal_plugin;

import gov.dot.fhwa.saxton.carma.guidance.util.trajectoryconverter.RoutePointStamped;
import gov.dot.fhwa.saxton.carma.signal_plugin.ead.IMotionPredictor;
import gov.dot.fhwa.saxton.carma.signal_plugin.ead.SimpleNCVMotionPredictor;

import java.util.Arrays;
import java.util.List;
import java.util.ArrayList;
import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;
import org.junit.Before;
import org.junit.Test;
import org.ros.message.MessageFactory;
import org.ros.message.Time;
import org.ros.node.NodeConfiguration;

import cav_msgs.RoadwayObstacle;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

/**
 * Runs unit tests for the SimpleNCVMotionPredictor class
 */
public class SimpleNCVMotionPredictorTest {

  Log log;
  NodeConfiguration nodeConfiguration = NodeConfiguration.newPrivate();
  MessageFactory messageFactory = nodeConfiguration.getTopicMessageFactory();

  @Before
  public void setUp() throws Exception {
    log = LogFactory.getLog(SimpleNCVMotionPredictor.class);
    log.info("Setting up tests for SimpleNCVMotionPredictor");
  }

  /**
   * Tests the simple average speed prediction of measured vehicle position
   * @throws Exception
   */
  @Test
  public void testPredictMotion() throws Exception {
	IMotionPredictor motionPredictor = new SimpleNCVMotionPredictor();

    // Empty list will return empty list
    List<RoutePointStamped> points = motionPredictor.predictMotion("1", new ArrayList<RoadwayObstacle>(), 0.2, 1.0);
    assertTrue(points.isEmpty());
    
    // A single node will return a node
    RoadwayObstacle n1 = newRoadwayObstacle(10.0, 1.0, 1.0);
    points = motionPredictor.predictMotion("1", Arrays.asList(n1), 0.2, 1.0);
    assertEquals(6, points.size());
    assertTrue(routePointAlmostEqual(new RoutePointStamped(10.0, 0.0, 1.0), points.get(0), 0.0001));
    assertTrue(routePointAlmostEqual(new RoutePointStamped(10.2, 0.0, 1.2), points.get(1), 0.0001));
    assertTrue(routePointAlmostEqual(new RoutePointStamped(10.4, 0.0, 1.4), points.get(2), 0.0001));
    assertTrue(routePointAlmostEqual(new RoutePointStamped(10.6, 0.0, 1.6), points.get(3), 0.0001));
    assertTrue(routePointAlmostEqual(new RoutePointStamped(10.8, 0.0, 1.8), points.get(4), 0.0001));
    assertTrue(routePointAlmostEqual(new RoutePointStamped(11.0, 0.0, 2.0), points.get(5), 0.0001));
    
    // 2 Node list with steady acceleration
    n1 = newRoadwayObstacle(14.0, 0.0, 0.8);
    RoadwayObstacle n2 = newRoadwayObstacle(15.0, 1.0, 1.2);
    points = motionPredictor.predictMotion("1", Arrays.asList(n1,n2), 1.0, 5.0);
    assertEquals(6, points.size());
    assertTrue(routePointAlmostEqual(new RoutePointStamped(15.0, 0.0, 1.0), points.get(0), 0.0001));
    assertTrue(routePointAlmostEqual(new RoutePointStamped(16.0, 0.0, 2.0), points.get(1), 0.0001));
    assertTrue(routePointAlmostEqual(new RoutePointStamped(17.0, 0.0, 3.0), points.get(2), 0.0001));
    assertTrue(routePointAlmostEqual(new RoutePointStamped(18.0, 0.0, 4.0), points.get(3), 0.0001));
    assertTrue(routePointAlmostEqual(new RoutePointStamped(19.0, 0.0, 5.0), points.get(4), 0.0001));
    assertTrue(routePointAlmostEqual(new RoutePointStamped(20.0, 0.0, 6.0), points.get(5), 0.0001));

    // 2 Node list with 0 velocity
    n1 = newRoadwayObstacle(14.0, 0.0, 0.0);
    n2 = newRoadwayObstacle(15.0, 1.0, 0.0);
    points = motionPredictor.predictMotion("1", Arrays.asList(n1,n2), 1.0, 0.6);
    assertEquals(7, points.size());
    assertTrue(routePointAlmostEqual(new RoutePointStamped(15.0, 0.0, 1.0), points.get(0), 0.0001));
    assertTrue(routePointAlmostEqual(new RoutePointStamped(15.0, 0.0, 1.1), points.get(1), 0.0001));
    assertTrue(routePointAlmostEqual(new RoutePointStamped(15.0, 0.0, 1.2), points.get(2), 0.0001));
    assertTrue(routePointAlmostEqual(new RoutePointStamped(15.0, 0.0, 1.3), points.get(3), 0.0001));
    assertTrue(routePointAlmostEqual(new RoutePointStamped(15.0, 0.0, 1.4), points.get(4), 0.0001));
    assertTrue(routePointAlmostEqual(new RoutePointStamped(15.0, 0.0, 1.5), points.get(5), 0.0001));
    assertTrue(routePointAlmostEqual(new RoutePointStamped(15.0, 0.0, 1.6), points.get(6), 0.0001));
    
    // 5 Node list with complex motion
    n1 = newRoadwayObstacle(10.0, 0.0, 5.0);
    n2 = newRoadwayObstacle(40.0, 4.0, 10.0);
    RoadwayObstacle n3 = newRoadwayObstacle(80.0, 8.0, 10.0);
    RoadwayObstacle n4 = newRoadwayObstacle(110.0, 12.0, 5.0);
    RoadwayObstacle n5 = newRoadwayObstacle(130.0, 16.0, 5.0); // Finish building list
    points = motionPredictor.predictMotion("1",Arrays.asList(n1,n2,n3,n4,n5), 14.0, 8.0);
    assertEquals(5, points.size());
    assertTrue(routePointAlmostEqual(new RoutePointStamped(130, 0.0, 16.0), points.get(0), 0.0001));
    assertTrue(routePointAlmostEqual(new RoutePointStamped(144, 0.0, 18.0), points.get(1), 0.0001));
    assertTrue(routePointAlmostEqual(new RoutePointStamped(158, 0.0, 20.0), points.get(2), 0.0001));
    assertTrue(routePointAlmostEqual(new RoutePointStamped(172, 0.0, 22.0), points.get(3), 0.0001));
    assertTrue(routePointAlmostEqual(new RoutePointStamped(186, 0.0, 24.0), points.get(4), 0.0001));
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

  /** 
   * Helper function acts as constructor for RoadwayObstacle
   */
  RoadwayObstacle newRoadwayObstacle(double dist, double time, double speed) {
    RoadwayObstacle ro = messageFactory.newFromType(RoadwayObstacle._TYPE);
    ro.getObject().getHeader().setStamp(Time.fromMillis((long)(time * 1000)));
    ro.setDownTrack(dist);
    ro.getObject().getVelocity().getTwist().getLinear().setX(speed);

    return ro;
  }
}
