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

import static org.mockito.ArgumentMatchers.*;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import gov.dot.fhwa.saxton.carma.guidance.ArbitratorService;
import gov.dot.fhwa.saxton.carma.guidance.conflictdetector.ConflictManager;
import gov.dot.fhwa.saxton.carma.guidance.conflictdetector.IMobilityTimeProvider;
import gov.dot.fhwa.saxton.carma.guidance.params.ParameterSource;
import gov.dot.fhwa.saxton.carma.guidance.plugins.PluginServiceLocator;
import gov.dot.fhwa.saxton.carma.guidance.util.ILogger;
import gov.dot.fhwa.saxton.carma.guidance.util.ITimeProvider;
import gov.dot.fhwa.saxton.carma.guidance.util.LoggerManager;
import gov.dot.fhwa.saxton.carma.guidance.util.RouteService;
import gov.dot.fhwa.saxton.carma.guidance.util.SaxtonLoggerProxyFactory;
import gov.dot.fhwa.saxton.carma.guidance.util.trajectoryconverter.RoutePointStamped;
import gov.dot.fhwa.saxton.carma.route.RouteSegment;
import gov.dot.fhwa.saxton.carma.route.Route;
import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.IGlidepathAppConfig;
import gov.dot.fhwa.saxton.carma.signal_plugin.ead.IMotionInterpolator;
import gov.dot.fhwa.saxton.carma.signal_plugin.ead.PlanInterpolator;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.ArrayList;
import org.apache.commons.logging.LogFactory;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import org.ros.message.MessageFactory;
import org.ros.message.Time;
import org.ros.node.NodeConfiguration;

import cav_msgs.RoadwayObstacle;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import gov.dot.fhwa.saxton.carma.signal_plugin.ead.trajectorytree.Node;

/**
 * Runs unit tests for the ObjectCollisionChecker class
 */
public class ObjectCollisionCheckerTest {

  SaxtonLoggerProxyFactory loggerFactory;
  ILogger log;
  NodeConfiguration nodeConfiguration = NodeConfiguration.newPrivate();
  MessageFactory messageFactory = nodeConfiguration.getTopicMessageFactory();

  @Before
  public void setUp() throws Exception {
    loggerFactory = new SaxtonLoggerProxyFactory(LogFactory.getLog(ObjectCollisionCheckerTest.class));
    LoggerManager.setLoggerFactory(loggerFactory);
    log = LoggerManager.getLogger();
    log.info("Setting up tests for PlanInterpolator");
  }

  @After
  public void tearDown() throws Exception {
  }

  /**
   * Tests basic ncv collision detection in the ObjectCollisionChecker
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
    IMobilityTimeProvider mobilityTimeProvider = mock(IMobilityTimeProvider.class);
    ArbitratorService           as = mock(ArbitratorService.class);
    Route                       route = mock(Route.class);
    RouteSegment mockSegment = mock(RouteSegment.class);
    List<RouteSegment> routeSegments = new ArrayList<>(Collections.nCopies(200, mockSegment)); // Create route full of the mocked segment
    when(route.getSegments()).thenReturn(routeSegments);
    when(mockSegment.determinePrimaryLane(anyDouble())).thenReturn(0);

    // Create conflict manager using default guidance settings

    ConflictManager cm = new ConflictManager(new double[] {10.0, 6.0, 0.2}, 5.0, 1.2, 0.1,
      0.0, -0.25, 0.0, mobilityTimeProvider);

    cm.setRoute(route);

    when(psl.getParameterSource()).thenReturn(ps);
    when(psl.getRouteService()).thenReturn(rs);
    when(psl.getTimeProvider()).thenReturn(tp);
    when(psl.getConflictDetector()).thenReturn(cm);
    when(psl.getArbitratorService()).thenReturn(as);

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

    ObjectCollisionChecker occ = new ObjectCollisionChecker(psl, motionPredictorFactory, planInterpolator);

    // Check no collisions without object data
    // Note: Nodes in this test are defined using double constructor
    Node n1 = new Node(0.0, 0.0, 5.0);
    Node n2 = new Node(5.0, 1.0, 5.0);
    assertFalse(occ.hasCollision(Arrays.asList(n1,n2), 0, 0));

    // Return the current lane id as 0 with crosstrack 0.0
    when(rs.getCurrentCrosstrackDistance()).thenReturn(0.0);
    when(rs.getCurrentRouteSegment()).thenReturn(mockSegment);
    double currentDowntrack = 0.0;
    when(rs.getCurrentDowntrackDistance()).thenReturn(currentDowntrack);

    long currentTime = 510L; // The current time is 0.51
    when(tp.getCurrentTimeMillis()).thenReturn(currentTime);
    when(mobilityTimeProvider.getCurrentTimeMillis()).thenReturn(currentTime);

    // Add one obstacle in current lane
    RoadwayObstacle ro = newRoadwayObstacle(0, 2.5, 0.5, 5.0); // Object id 0 with stamp at 0.5
    ro.setPrimaryLane((byte)0);
    occ.updateObjects(Arrays.asList(ro));

    assertEquals(1, occ.trackedLaneObjectsHistory.size()); // Object id is being tracked
    assertEquals(1, occ.trackedLaneObjectsPredictions.size()); // Object id is being tracked
    assertEquals(1, occ.trackedLaneObjectsHistory.get(0).size()); // 1 history element 
    assertEquals(0, occ.trackedLaneObjectsPredictions.get(0).size()); // 0 predictions

    // Check that the collision is not found since there is no prediction
    assertFalse(occ.hasCollision(Arrays.asList(n1,n2), 0, 0));

    currentTime = 610L; // The current time is 0.61

    // Obstacle is sampled a second time
    RoadwayObstacle ro2 = newRoadwayObstacle(0, 3.0, 0.6, 5.0); // Object id 0 with stamp at 0.6
    ro.setPrimaryLane((byte)0);
    occ.updateObjects(Arrays.asList(ro2));

    assertEquals(1, occ.trackedLaneObjectsHistory.size()); // Object id is being tracked
    assertEquals(1, occ.trackedLaneObjectsPredictions.size()); // Object id is being tracked
    assertEquals(2, occ.trackedLaneObjectsHistory.get(0).size()); // 1 history element 
    assertEquals(7, occ.trackedLaneObjectsPredictions.get(0).size()); // 1 predictions

    // Check that the collision is found
    assertTrue(occ.hasCollision(Arrays.asList(n1,n2), 0, 0));

    // Set the host plan with collisions
    occ.setHostPlan(Arrays.asList(n1,n2), 0, 0);
    // Obstacle is sampled a third time
    ro2 = newRoadwayObstacle(0, 3.5, 0.7, 5.0); // Object id 0 with stamp at 0.6
    ro.setPrimaryLane((byte)0);

    currentTime = 710L; // The current time is 0.71
    occ.updateObjects(Arrays.asList(ro2)); // Call to trigger replan
    
    verify(as, times(1)).requestNewPlan(); // Verify replan occurred

    // Set the host plan without collisions
    n1 = new Node(0.0, 4.0, 0.0);
    n2 = new Node(0.0, 5.0, 0.0);
    occ.setHostPlan(Arrays.asList(n1,n2), 0, 0);

    // Obstacle is sampled a fourth time
    ro2 = newRoadwayObstacle(0, 4, 0.8, 5.0); // Object id 0 with stamp at 0.6
    ro.setPrimaryLane((byte)0);

    currentTime = 810L; // The current time is 0.81
    occ.updateObjects(Arrays.asList(ro2)); // Call to trigger replan
    
    verify(as, times(1)).requestNewPlan(); // Verify replan did not occur

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
  RoadwayObstacle newRoadwayObstacle(int id, double dist, double time, double speed) {
    RoadwayObstacle ro = messageFactory.newFromType(RoadwayObstacle._TYPE);
    ro.getObject().setId(id);
    ro.getObject().getHeader().setStamp(Time.fromMillis((long)(time * 1000)));
    ro.setDownTrack(dist);
    ro.getObject().getVelocity().getTwist().getLinear().setX(speed);

    return ro;
  }
}
