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

import static org.mockito.ArgumentMatchers.any;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import gov.dot.fhwa.saxton.carma.geometry.GeodesicCartesianConverter;
import gov.dot.fhwa.saxton.carma.geometry.cartesian.Point3D;
import gov.dot.fhwa.saxton.carma.geometry.cartesian.Vector;
import gov.dot.fhwa.saxton.carma.geometry.cartesian.Vector3D;
import gov.dot.fhwa.saxton.carma.geometry.geodesic.Location;
import gov.dot.fhwa.saxton.carma.guidance.TrackingService;
import gov.dot.fhwa.saxton.carma.guidance.conflictdetector.IConflictDetector;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.IManeuverInputs;
import gov.dot.fhwa.saxton.carma.guidance.mobilityrouter.IMobilityRouter;
import gov.dot.fhwa.saxton.carma.guidance.params.ParameterSource;
import gov.dot.fhwa.saxton.carma.guidance.plugins.PluginServiceLocator;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.IPublisher;
import gov.dot.fhwa.saxton.carma.guidance.util.ILogger;
import gov.dot.fhwa.saxton.carma.guidance.util.ITimeProvider;
import gov.dot.fhwa.saxton.carma.guidance.util.RouteService;
import gov.dot.fhwa.saxton.carma.guidance.util.trajectoryconverter.RoutePointStamped;
import gov.dot.fhwa.saxton.carma.rosutils.SaxtonLogger;
import gov.dot.fhwa.saxton.carma.route.RouteSegment;
import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.IGlidepathAppConfig;
import gov.dot.fhwa.saxton.carma.signal_plugin.ead.IMotionInterpolator;
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
import org.ros.message.Time;
import org.ros.node.NodeConfiguration;
import org.ros.rosjava_geometry.Transform;

import cav_msgs.RoadwayObstacle;
import sensor_msgs.NavSatFix;
import sensor_msgs.NavSatStatus;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import gov.dot.fhwa.saxton.carma.signal_plugin.ead.trajectorytree.Node;

/**
 * Runs unit tests for the ObjectCollisionChecker class
 */
public class ObjectCollisionCheckerTest {

  SaxtonLogger log;
  NodeConfiguration nodeConfiguration = NodeConfiguration.newPrivate();
  MessageFactory messageFactory = nodeConfiguration.getTopicMessageFactory();

  @Before
  public void setUp() throws Exception {
    log = new SaxtonLogger("ObjectCollisionChecker", LogFactory.getLog(ObjectCollisionChecker.class));
    log.info("Setting up tests for ObjectCollisionChecker");
  }

  @After
  public void tearDown() throws Exception {
  }

  /**
   * Tests the linear regression of measured vehicle position vs time.
   * @throws Exception
   */
  @Test
  public void testUpdateObjects() throws Exception {

    IMotionInterpolator planInterpolator = new PlanInterpolator();
    IMotionPredictorModelFactory motionPredictorFactory = new DefaultMotionPredictorFactory(mock(IGlidepathAppConfig.class));

    PluginServiceLocator        psl = mock(PluginServiceLocator.class);
    ParameterSource             ps = mock(ParameterSource.class);
    RouteService                rs = mock(RouteService.class);
    ITimeProvider               tp = mock(ITimeProvider.class);
    IConflictDetector           cd = mock(IConflictDetector.class);

    // TODO how do we check that the conflict was found

    when(psl.getParameterSource()).thenReturn(ps);
    when(psl.getRouteService()).thenReturn(rs);
    when(psl.getTimeProvider()).thenReturn(tp);
    when(psl.getConflictDetector()).thenReturn(cd);

    // PluginServiceLocator psl = new PluginServiceLocator(mock(ArbitratorService.class),    mock(PluginManagementService.class),
    //                                                 mock(IPubSubService.class),       mock(ParameterSource.class),
    //                                                 mock(ManeuverPlanner.class),      mockRouteService,
    //                                                 mock(IMobilityRouter.class),      mock(IConflictDetector.class),
    //                                                 mock(ITrajectoryConverter.class), mock(ILightBarManager.class),
    //                                                 mock(TrackingService.class));


    
    when(ps.getString("ead.NCVHandling.objectMotionPredictorModel")).thenReturn("SIMPLE_LINEAR_REGRESSION");
    when(ps.getInteger("ead.NCVHandling.collision.maxObjectHistoricalDataAge")).thenReturn(3000);
    when(ps.getDouble("ead.NCVHandling.collision.distanceStep")).thenReturn(2.5);
    when(ps.getDouble("ead.NCVHandling.collision.timeDuration")).thenReturn(3.0);
    when(ps.getDouble("ead.NCVHandling.collision.downtrackBuffer")).thenReturn(8.0);
    when(ps.getDouble("ead.NCVHandling.collision.crosstrackBuffer")).thenReturn(2.0);
    when(ps.getDouble("ead.NCVHandling.collision.timeMargin")).thenReturn(0.2);
    when(ps.getDouble("ead.NCVHandling.collision.longitudinalBias")).thenReturn(0.0);
    when(ps.getDouble("ead.NCVHandling.collision.lateralBias")).thenReturn(0.0);
    when(ps.getDouble("ead.NCVHandling.collision.temporalBias")).thenReturn(0.0);

    ObjectCollisionChecker occ = new ObjectCollisionChecker(psl, log, motionPredictorFactory, planInterpolator);

    // Check no collisions without object data
    // Note: Nodes in this test are defined using double constructor
    Node n1 = new Node(0.0, 0.0, 5.0);
    Node n2 = new Node(5.0, 1.0, 5.0);
    assertFalse(occ.hasCollision(Arrays.asList(n1,n2)));

    // Return the current lane id as 0 with crosstrack 0.0
    when(rs.getCurrentCrosstrackDistance()).thenReturn(0.0);
    RouteSegment mockSegment = mock(RouteSegment.class);
    when(rs.getCurrentRouteSegment()).thenReturn(mockSegment);
    when(mockSegment.determinePrimaryLane(0.0)).thenReturn(0);

    when(tp.getCurrentTimeMillis()).thenReturn(510L); // The current time is 0.51

    // Add one obstacle in current lane
    RoadwayObstacle ro = newRoadwayObstacle(2.5, 0.5, 5.0); // Object with stamp at 0.5
    ro.setPrimaryLane((byte)0);
    occ.updateObjects(Arrays.asList(ro));

    assertEquals(1, occ.trackedLaneObjectsHistory.size());
    assertEquals(1, occ.trackedLaneObjectsPredictions.size());

    // Check that the collision is found
    assertTrue(occ.hasCollision(Arrays.asList(n1,n2)));


    // // Empty list will return empty list
    // List<RoutePointStamped> points = motionPredictor.predictMotion("1", new ArrayList<Node>(), 0.2, 1.0);
    // assertTrue(points.isEmpty());

    // // A single node will return an empty list
    // Node n1 = new Node(10.0, 0.0, 5.0);
    // points = motionPredictor.predictMotion("1", Arrays.asList(n1), 0.2, 1.0);
    // assertTrue(points.isEmpty());

    // // 2 Node list with 0 acceleration
    // n1 = new Node(10.0, 0.0, 5.0);
    // Node n2 = new Node(15.0, 1.0, 5.0);
    // points = motionPredictor.predictMotion("1", Arrays.asList(n1,n2), 1.0, 1.0);

    // assertEquals(5, points.size());
    // assertTrue(routePointAlmostEqual(new RoutePointStamped(16.0, 0.0, 1.2), points.get(0), 0.0001));
    // assertTrue(routePointAlmostEqual(new RoutePointStamped(17.0, 0.0, 1.4), points.get(1), 0.0001));
    // assertTrue(routePointAlmostEqual(new RoutePointStamped(18.0, 0.0, 1.6), points.get(2), 0.0001));
    // assertTrue(routePointAlmostEqual(new RoutePointStamped(19.0, 0.0, 1.8), points.get(3), 0.0001));
    // assertTrue(routePointAlmostEqual(new RoutePointStamped(20.0, 0.0, 2.0), points.get(4), 0.0001));

    // // 5 Node list with complex motion
    // n1 = new Node(10.0, 0.0, 5.0);
    // n2 = new Node(40.0, 4.0, 10.0);
    // Node n3 = new Node(80.0, 8.0, 10.0);
    // Node n4 = new Node(110.0, 12.0, 5.0);
    // Node n5 = new Node(130.0, 16.0, 5.0); // Finish building list
    // points = motionPredictor.predictMotion("1",Arrays.asList(n1,n2,n3,n4,n5), 10.0, 8.0);

    // assertEquals(7,points.size());
    // assertTrue(routePointAlmostEqual(new RoutePointStamped(140, 0.0, 16.41975), points.get(0), 0.0001));
    // assertTrue(routePointAlmostEqual(new RoutePointStamped(150, 0.0, 17.69547), points.get(1), 0.0001));
    // assertTrue(routePointAlmostEqual(new RoutePointStamped(160, 0.0, 18.97119), points.get(2), 0.0001));
    // assertTrue(routePointAlmostEqual(new RoutePointStamped(170, 0.0, 20.24691), points.get(3), 0.0001));
    // assertTrue(routePointAlmostEqual(new RoutePointStamped(180, 0.0, 21.52263), points.get(4), 0.0001));
    // assertTrue(routePointAlmostEqual(new RoutePointStamped(190, 0.0, 22.79835), points.get(5), 0.0001));
    // assertTrue(routePointAlmostEqual(new RoutePointStamped(200, 0.0, 24.07407), points.get(6), 0.0001));
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
