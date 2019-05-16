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

package gov.dot.fhwa.saxton.carma.signal_plugin.ead.trajectorytree;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;
import static org.mockito.ArgumentMatchers.*;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.ThreadLocalRandom;
import java.util.concurrent.TimeUnit;

import org.junit.Before;
import org.junit.Test;
import org.mockito.Mockito;
import org.ros.message.MessageFactory;
import org.ros.message.Time;
import org.ros.node.NodeConfiguration;

import gov.dot.fhwa.saxton.carma.geometry.cartesian.spatialstructure.NSpatialHashMapFactory;
import gov.dot.fhwa.saxton.carma.guidance.ArbitratorService;
import gov.dot.fhwa.saxton.carma.guidance.conflictdetector.ConflictManager;
import gov.dot.fhwa.saxton.carma.guidance.conflictdetector.IConflictDetector;
import gov.dot.fhwa.saxton.carma.guidance.conflictdetector.IMobilityTimeProvider;
import gov.dot.fhwa.saxton.carma.guidance.params.ParameterSource;
import gov.dot.fhwa.saxton.carma.guidance.plugins.PluginServiceLocator;
import gov.dot.fhwa.saxton.carma.guidance.util.ILogger;
import gov.dot.fhwa.saxton.carma.guidance.util.ILoggerFactory;
import gov.dot.fhwa.saxton.carma.guidance.util.LoggerManager;
import gov.dot.fhwa.saxton.carma.guidance.util.RouteService;
import gov.dot.fhwa.saxton.carma.guidance.util.trajectoryconverter.RoutePointStamped;
import gov.dot.fhwa.saxton.carma.route.Route;
import gov.dot.fhwa.saxton.carma.route.RouteSegment;
import gov.dot.fhwa.saxton.carma.guidance.util.ITimeProvider;
import gov.dot.fhwa.saxton.carma.signal_plugin.IReplanHandle;
import gov.dot.fhwa.saxton.carma.signal_plugin.NoOpCollisionChecker;
import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.IGlidepathAppConfig;
import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.SignalPhase;
import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.utils.GlidepathApplicationContext;
import gov.dot.fhwa.saxton.carma.signal_plugin.asd.IntersectionData;
import gov.dot.fhwa.saxton.carma.signal_plugin.asd.map.MapMessage;
import gov.dot.fhwa.saxton.carma.signal_plugin.ead.EadAStar;
import gov.dot.fhwa.saxton.carma.signal_plugin.ead.IMotionInterpolator;
import gov.dot.fhwa.saxton.carma.signal_plugin.ead.INodeCollisionChecker;
import gov.dot.fhwa.saxton.carma.signal_plugin.ead.IntersectionGeometry;
import gov.dot.fhwa.saxton.carma.signal_plugin.ead.PlanInterpolator;
import gov.dot.fhwa.saxton.carma.signal_plugin.ead.trajectorytree.AStarSolver;
import gov.dot.fhwa.saxton.carma.signal_plugin.ead.trajectorytree.FinePathNeighbors;
import gov.dot.fhwa.saxton.carma.signal_plugin.ead.trajectorytree.Node;
import cav_msgs.RoadwayObstacle;

/**
 * !!!!!!!!!!!!!!!!!!!!!!!!!!!
 * TODO Make these unit tests
 * !!!!!!!!!!!!!!!!!!!!!!!!!!!
 */


public class FinePathNeighborsTest {
    
    IGlidepathAppConfig mockConfig = mock(IGlidepathAppConfig.class, Mockito.withSettings().stubOnly());
    final double timeBuffer = 4.0;
    NodeConfiguration nodeConfiguration = NodeConfiguration.newPrivate();
    MessageFactory messageFactory = nodeConfiguration.getTopicMessageFactory();
    public static long PLAN_START_TIME = 0; //TODO remove
    IReplanHandle mockReplanHandle;
    
    @Before
    public void setup() {
        ILoggerFactory mockFact = mock(ILoggerFactory.class, Mockito.withSettings().stubOnly());
        ILogger mockLogger = mock(ILogger.class, Mockito.withSettings().stubOnly());
        when(mockFact.createLoggerForClass(any())).thenReturn(mockLogger);
        LoggerManager.setLoggerFactory(mockFact);
        GlidepathApplicationContext.getInstance().setAppConfigOverride(mockConfig);
        when(mockConfig.getDoubleDefaultValue("defaultAccel", 2.0)).thenReturn(1.0);
        when(mockConfig.getMaximumSpeed(0.0)).thenReturn(25);
        when(mockConfig.getDoubleDefaultValue("crawlingSpeed", 5.0)).thenReturn(5.0);
        when(mockConfig.getDoubleDefaultValue("ead.timebuffer", 4.0)).thenReturn(timeBuffer);
        when(mockConfig.getDoubleDefaultValue("ead.response.lag", 1.9)).thenReturn(0.0); 
        when(mockConfig.getDoubleDefaultValue("ead.acceptableStopDistance", 6.0)).thenReturn(6.0);
        when(mockConfig.getDoubleDefaultValue("ead.debugThreshold", -1.0)).thenReturn(-1.0);
        when(mockConfig.getMaximumSpeed(0.0)).thenReturn(25);
        
        mockReplanHandle = mock(IReplanHandle.class, Mockito.withSettings().stubOnly());
    }


    @Test
    public void testGetViableSpeeds() {
      when(mockConfig.getDoubleDefaultValue("defaultAccel", 2.0)).thenReturn(1.0);
      when(mockConfig.getMaximumSpeed(0.0)).thenReturn(25);
      when(mockConfig.getDoubleDefaultValue("crawlingSpeed", 5.0)).thenReturn(2.23694);

      FinePathNeighbors fpn = new FinePathNeighbors();

      IntersectionData intersection1 = new IntersectionData(); // Id 9945
      intersection1.setMap(mock(MapMessage.class, Mockito.withSettings().stubOnly()));
      intersection1.setRoughDist(8423); 
      intersection1.setDtsb(100.0);
      intersection1.setCurrentPhase(SignalPhase.GREEN);
      intersection1.setTimeToNextPhase(26.99095117187494);
      intersection1.setStopBoxWidth(35.18);
      intersection1.setIntersectionId(9945);
      intersection1.setGeometry(new IntersectionGeometry(40, 100));
      List<IntersectionData> intersections = Arrays.asList(intersection1);

      double timeIncrement = 1.0;
      double speedIncrement = 1.0;
      fpn.initialize(intersections, 1, timeIncrement, speedIncrement, new NoOpCollisionChecker(), 0, 0);

      List<Double> speeds = fpn.getViableSpeeds(new Node(0,0,0), timeIncrement);

      assertEquals(2, speeds.size());
      assertEquals(1.0, speeds.get(0), 0.0001);
      assertEquals(1.0, speeds.get(1), 0.0001);

      // Make crawling speed larger than max speed to check capping behavior
      when(mockConfig.getDoubleDefaultValue("crawlingSpeed", 5.0)).thenReturn(5.0);
      fpn = new FinePathNeighbors();
      fpn.initialize(intersections, 1, timeIncrement, speedIncrement, new NoOpCollisionChecker(), 0, 0);

      speeds = fpn.getViableSpeeds(new Node(0,0,0), timeIncrement);

      assertEquals(2, speeds.size());
      assertEquals(2.2352, speeds.get(0), 0.0001);
      assertEquals(1.0, speeds.get(1), 0.0001);


      // Check full speed range
      when(mockConfig.getDoubleDefaultValue("crawlingSpeed", 5.0)).thenReturn(2.23694);
      when(mockConfig.getDoubleDefaultValue("defaultAccel", 2.0)).thenReturn(2.0);
      fpn = new FinePathNeighbors();
      fpn.initialize(intersections, 1, timeIncrement, speedIncrement, new NoOpCollisionChecker(), 0, 0);

      speeds = fpn.getViableSpeeds(new Node(0,0,9), timeIncrement);

      assertEquals(5, speeds.size());
      assertEquals(7.0, speeds.get(0), 0.0001);
      assertEquals(9.0, speeds.get(1), 0.0001);
      assertEquals(8.0, speeds.get(2), 0.0001);
      assertEquals(10.0, speeds.get(3), 0.0001);
      assertEquals(11.0, speeds.get(4), 0.0001);

      // Check max speed cap
      when(mockConfig.getDoubleDefaultValue("crawlingSpeed", 5.0)).thenReturn(2.23694);
      when(mockConfig.getDoubleDefaultValue("defaultAccel", 2.0)).thenReturn(2.0);
      fpn = new FinePathNeighbors();
      fpn.initialize(intersections, 1, timeIncrement, speedIncrement, new NoOpCollisionChecker(), 0, 0);

      speeds = fpn.getViableSpeeds(new Node(0,0,10), timeIncrement);

      assertEquals(4, speeds.size());
      assertEquals(8.0, speeds.get(0), 0.0001);
      assertEquals(10.0, speeds.get(1), 0.0001);
      assertEquals(9.0, speeds.get(2), 0.0001);
      assertEquals(11.176, speeds.get(3), 0.0001);

      // Check max speed cap at current speed
      when(mockConfig.getDoubleDefaultValue("crawlingSpeed", 5.0)).thenReturn(2.23694);
      when(mockConfig.getDoubleDefaultValue("defaultAccel", 2.0)).thenReturn(2.0);
      fpn = new FinePathNeighbors();
      fpn.initialize(intersections, 1, timeIncrement, speedIncrement, new NoOpCollisionChecker(), 0, 0);

      speeds = fpn.getViableSpeeds(new Node(0,0,11), timeIncrement);

      assertEquals(4, speeds.size());
      assertEquals(9.0, speeds.get(0), 0.0001);
      assertEquals(11.0, speeds.get(1), 0.0001);
      assertEquals(10.0, speeds.get(2), 0.0001);
      assertEquals(11.176, speeds.get(3), 0.0001); // Max speed
      
      
      // Check min speed cap
      when(mockConfig.getDoubleDefaultValue("crawlingSpeed", 5.0)).thenReturn(5.0);
      when(mockConfig.getDoubleDefaultValue("defaultAccel", 2.0)).thenReturn(2.0);
      fpn = new FinePathNeighbors();
      fpn.initialize(intersections, 1, timeIncrement, speedIncrement, new NoOpCollisionChecker(), 0, 0);

      speeds = fpn.getViableSpeeds(new Node(0,0,3.0), timeIncrement);

      assertEquals(4, speeds.size());
      assertEquals(2.2352, speeds.get(0), 0.0001); // Min speed
      assertEquals(3.0, speeds.get(1), 0.0001);
      assertEquals(4.0, speeds.get(2), 0.0001); 
      assertEquals(5.0, speeds.get(3), 0.0001);   


      // Check Acceptable stopping distance on red
      when(mockConfig.getDoubleDefaultValue("crawlingSpeed", 5.0)).thenReturn(5.0);
      when(mockConfig.getDoubleDefaultValue("defaultAccel", 2.0)).thenReturn(2.0);
      fpn = new FinePathNeighbors();

      intersection1 = new IntersectionData(); // Id 9945
      intersection1.setMap(mock(MapMessage.class, Mockito.withSettings().stubOnly()));
      intersection1.setRoughDist(8423); 
      intersection1.setDtsb(5.0);
      intersection1.setCurrentPhase(SignalPhase.RED);
      intersection1.setTimeToNextPhase(26.99095117187494);
      intersection1.setStopBoxWidth(35.18);
      intersection1.setIntersectionId(9945);
      intersection1.setGeometry(new IntersectionGeometry(40, 100));
      intersections = Arrays.asList(intersection1);

      fpn.initialize(intersections, 1, timeIncrement, speedIncrement, new NoOpCollisionChecker(), 0, 0);

      speeds = fpn.getViableSpeeds(new Node(0,0,0), timeIncrement);

      assertEquals(1, speeds.size());
      assertEquals(0.0, speeds.get(0), 0.0001);


      // Check Acceptable stopping distance on green
      when(mockConfig.getDoubleDefaultValue("crawlingSpeed", 5.0)).thenReturn(5.0);
      when(mockConfig.getDoubleDefaultValue("defaultAccel", 2.0)).thenReturn(2.0);
      fpn = new FinePathNeighbors();

      intersection1 = new IntersectionData(); // Id 9945
      intersection1.setMap(mock(MapMessage.class, Mockito.withSettings().stubOnly()));
      intersection1.setRoughDist(8423); 
      intersection1.setDtsb(5.0);
      intersection1.setCurrentPhase(SignalPhase.GREEN);
      intersection1.setTimeToNextPhase(26.99095117187494);
      intersection1.setStopBoxWidth(35.18);
      intersection1.setIntersectionId(9945);
      intersection1.setGeometry(new IntersectionGeometry(40, 100));
      intersections = Arrays.asList(intersection1);

      fpn.initialize(intersections, 1, timeIncrement, speedIncrement, new NoOpCollisionChecker(), 0, 0);

      speeds = fpn.getViableSpeeds(new Node(0,0,0), timeIncrement);

      assertEquals(2, speeds.size());
      assertEquals(2.2352, speeds.get(0), 0.0001);
      assertEquals(2.0, speeds.get(1), 0.0001);
  }

  /** 
   * Helper function acts as constructor for RoadwayObstacle
   */
  RoadwayObstacle newRoadwayObstacle(int id, double dist, double time, double speed) {
    RoadwayObstacle ro = messageFactory.newFromType(RoadwayObstacle._TYPE);
    ro.getObject().setId(id);
    ro.getObject().getHeader().setStamp(Time.fromMillis((long)(time * 1000.0)));
    ro.setDownTrack(dist);
    ro.getObject().getVelocity().getTwist().getLinear().setX(speed);

    return ro;
  }
    
}
