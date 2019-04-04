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
import org.junit.Ignore;
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
import gov.dot.fhwa.saxton.carma.signal_plugin.ead.trajectorytree.ANAStarSolver;
import gov.dot.fhwa.saxton.carma.signal_plugin.ead.trajectorytree.AStarSolver;
import gov.dot.fhwa.saxton.carma.signal_plugin.ead.trajectorytree.FinePathNeighbors;
import gov.dot.fhwa.saxton.carma.signal_plugin.ead.trajectorytree.ITreeSolver;
import gov.dot.fhwa.saxton.carma.signal_plugin.ead.trajectorytree.Node;
import cav_msgs.RoadwayObstacle;

public class EADAStarPlanTest {
    
    IGlidepathAppConfig mockConfig = mock(IGlidepathAppConfig.class, Mockito.withSettings().stubOnly());
    INodeCollisionChecker mockCC = new MockCollisionChecker();
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
        when(mockConfig.getDoubleDefaultValue("defaultAccel", 2.0)).thenReturn(2.0);
        when(mockConfig.getDoubleValue("defaultAccel")).thenReturn(2.0);
        when(mockConfig.getMaximumSpeed(0.0)).thenReturn(25);
        when(mockConfig.getDoubleDefaultValue("crawlingSpeed", 5.0)).thenReturn(5.0);
        when(mockConfig.getDoubleDefaultValue("ead.timebuffer", 4.0)).thenReturn(timeBuffer);
        when(mockConfig.getDoubleDefaultValue("ead.response.lag", 1.9)).thenReturn(0.0); // Was 1.9
        when(mockConfig.getProperty("ead.desiredCostModel")).thenReturn("MOVES_2010");
        when(mockConfig.getDoubleValue("ead.MOVES.rollingTermA")).thenReturn(0.22112); 
        when(mockConfig.getDoubleValue("ead.MOVES.rotatingTermB")).thenReturn(0.002838); 
        when(mockConfig.getDoubleValue("ead.MOVES.dragTermC")).thenReturn(0.000698); 
        when(mockConfig.getDoubleValue("ead.MOVES.vehicleMassInTons")).thenReturn(1.86686); 
        when(mockConfig.getDoubleValue("ead.MOVES.fixedMassFactor")).thenReturn(1.86686); 
        when(mockConfig.getProperty("ead.MOVES.baseRateTablePath")).thenReturn("../launch/params/BaseRateForPassengerTruck.csv");
        
        when(mockConfig.getDoubleValue("ead.MOVES.fuelNormalizationDenominator")).thenReturn(211859.0); 
        when(mockConfig.getDoubleValue("ead.MOVES.timeNormalizationDenominator")).thenReturn(1.0); 
        when(mockConfig.getDoubleValue("ead.MOVES.heuristicWeight")).thenReturn(1.0); 
        when(mockConfig.getDoubleValue("ead.MOVES.percentTimeCost")).thenReturn(0.5); 
        when(mockConfig.getMaximumSpeed(0.0)).thenReturn(25);
        
        mockReplanHandle = mock(IReplanHandle.class, Mockito.withSettings().stubOnly());
    }

    // These default phases will be used for phases which are not specified. Check NeighborBase.java to confirm.
    // protected final double                  DEFAULT_GREEN_DURATION = 27.0;  //sec
    // protected final double                  DEFAULT_YELLOW_DURATION = 3.0;  //sec
    // protected final double                  DEFAULT_RED_DURATION = 30.0;    //sec

    @Test
    public void planSingleIntersection() {
        when(mockConfig.getDoubleDefaultValue("ead.coarse_time_inc", 5.0)).thenReturn(2.0);
        when(mockConfig.getDoubleDefaultValue("ead.coarse_speed_inc", 3.0)).thenReturn(2.0);
        when(mockConfig.getDoubleDefaultValue("ead.fine_time_inc", 2.0)).thenReturn(2.0);
        when(mockConfig.getDoubleDefaultValue("ead.fine_speed_inc", 1.0)).thenReturn(1.0);
        when(mockConfig.getDoubleDefaultValue("ead.acceptableStopDistance", 6.0)).thenReturn(6.0);
        EadAStar ead = new EadAStar(mockCC);
        ANAStarSolver solver = new ANAStarSolver();
        solver.setMaxPlanningTimeMS(Long.MAX_VALUE);
        ead.initialize(1, solver);
        // AStarSolver s = new AStarSolver();
        // ead.initialize(1, s);
        IntersectionData intersection1 = new IntersectionData(); // Id 9945
        intersection1.setMap(mock(MapMessage.class, Mockito.withSettings().stubOnly()));
        intersection1.setRoughDist(1581); 
        intersection1.setDtsb(50);
        intersection1.setCurrentPhase(SignalPhase.GREEN);
        intersection1.setTimeToNextPhase(8.631097656249949);
        intersection1.setStopBoxWidth(32.90);
        intersection1.setIntersectionId(9709);
        intersection1.setGeometry(new IntersectionGeometry(40, 100));
        List<IntersectionData> intersections = Arrays.asList(intersection1);
        try {
            long startTime = System.currentTimeMillis();
            List<Node> res = ead.plan(0, 11.176, intersections, 0, 0);
            System.out.println("A* Planning for one intersections takes " + (System.currentTimeMillis() - startTime) + " ms to finish");
            for(Node n : res) {
                System.out.println(n.toString());
            }
        } catch(Exception e) {
            e.printStackTrace();
            assertTrue(false); // indicate the failure
        }
    }

    @Test
    public void planTwoIntersections() {
        when(mockConfig.getDoubleDefaultValue("ead.coarse_time_inc", 5.0)).thenReturn(2.0);
        when(mockConfig.getDoubleDefaultValue("ead.coarse_speed_inc", 3.0)).thenReturn(2.0);
        when(mockConfig.getDoubleDefaultValue("ead.fine_time_inc", 2.0)).thenReturn(2.0);
        when(mockConfig.getDoubleDefaultValue("ead.fine_speed_inc", 1.0)).thenReturn(1.0);
        when(mockConfig.getDoubleDefaultValue("ead.acceptableStopDistance", 6.0)).thenReturn(6.0);
        long startTime = System.currentTimeMillis();
        EadAStar ead = new EadAStar(mockCC);
        ANAStarSolver solver = new ANAStarSolver();
        solver.setMaxPlanningTimeMS(Long.MAX_VALUE);
        ead.initialize(1, solver);
        // ead.initialize(1, new AStarSolver());
        IntersectionData intersection1 = new IntersectionData(); // Id 9709
        intersection1.setMap(mock(MapMessage.class, Mockito.withSettings().stubOnly()));
        intersection1.setRoughDist(5700);
        intersection1.setDtsb(40.49);
        intersection1.setCurrentPhase(SignalPhase.RED);
        intersection1.setTimeToNextPhase(29.82590234374993);
        intersection1.setStopBoxWidth(32.90);
        intersection1.setIntersectionId(9709);
        intersection1.setGeometry(new IntersectionGeometry(40, 100));
        IntersectionData intersection2 = new IntersectionData(); // Id 9945
        intersection2.setMap(mock(MapMessage.class, Mockito.withSettings().stubOnly()));
        intersection2.setRoughDist(22394);
        intersection2.setDtsb(211.48);
        intersection2.setCurrentPhase(SignalPhase.RED);
        intersection2.setTimeToNextPhase(4.3409023437498035);
        intersection2.setStopBoxWidth(35.18);
        intersection2.setIntersectionId(9945);
        intersection2.setGeometry(new IntersectionGeometry(40, 100));
        List<IntersectionData> intersections = Arrays.asList(intersection1, intersection2);
        try {
            List<Node> res = ead.plan(1.9352529452127594, 11.176, intersections, 0, 0);
            System.out.println("A* Planning for two intersections takes " + (System.currentTimeMillis() - startTime) + " ms to finish");
            for(Node n : res) {
                System.out.println(n.toString());
            }
        } catch(Exception e) {
            e.printStackTrace();
            assertTrue(false); // indicate the failure
        }
    }

    /**
     * Tests many permutations of starting distance, signal phase, and phase timing to stress test EadAStar for two intersections
     * The test will fail if even a single plan fails
     * 
     * Note: This test may take a long time to run and should be tweaked as needed
     */
    @Test
    @Ignore("Ignore large test in CI system")
    public void planManyCases() {
        when(mockConfig.getDoubleDefaultValue("ead.coarse_time_inc", 5.0)).thenReturn(2.0);
        when(mockConfig.getDoubleDefaultValue("ead.coarse_speed_inc", 3.0)).thenReturn(2.0);
        when(mockConfig.getDoubleDefaultValue("ead.fine_time_inc", 2.0)).thenReturn(1.0);
        when(mockConfig.getDoubleDefaultValue("ead.fine_speed_inc", 1.0)).thenReturn(1.0);
        when(mockConfig.getDoubleDefaultValue("ead.acceptableStopDistance", 6.0)).thenReturn(6.0);
        int totalCount = 0;
        int failureCount = 0;
        long totalPlanningTime = 0;
        SignalPhase phase1 = SignalPhase.GREEN;
        SignalPhase phase2 = SignalPhase.GREEN;
        EadAStar ead;
        ANAStarSolver solver;
        long totalIterationCount = 0;
        //AStarSolver solver;
        for (double dist1 = 20; dist1 < 100; dist1 += 20.0) {
            for (double dist2 = dist1 + 80; dist2 < 250; dist2 += 20.0) {
                phase1 = SignalPhase.GREEN;
                while (phase1 != SignalPhase.NONE) {
                    phase2 = SignalPhase.GREEN;
                    while (phase2 != SignalPhase.NONE) {
                        for (double i = 0; i <  30.0; i+=5) {
                            for (double j = 0; j <  30.0; j+=5) {
                                if ((phase1 == SignalPhase.GREEN || phase2 == SignalPhase.GREEN) && (i < timeBuffer * 3.0 || j < timeBuffer * 3.0)) {
                                    //System.out.println("Ignoring impossible timing's dues to time buffer");
                                    continue;
                                }
                                //System.out.println("DTSB1: " + dist1 + " DTSB2: " + dist2 + " Phase1: " + phase1 + " Phase2: " + phase2 + " timeToNext1: " + i + " timeToNext2: " + j);
                                //The following code is for NCV handling
                                int randomPredictionStartLoc = ThreadLocalRandom.current().nextInt((int)dist1, (int)dist2 + 1);
                                int randomPredictionStartTime = 0;
                                int randomPredictionSpeed = ThreadLocalRandom.current().nextInt(1, 11);
                                int predictionPeriod = 5;
                                ((MockCollisionChecker) mockCC).setPredictedTrajectory(
                                        new Node(randomPredictionStartLoc, randomPredictionStartTime, randomPredictionSpeed),
                                        new Node(randomPredictionStartLoc + predictionPeriod * randomPredictionSpeed, randomPredictionStartTime + predictionPeriod, randomPredictionSpeed));
                                ead = new EadAStar(mockCC);
                                solver = new ANAStarSolver();
                                solver.setMaxPlanningTimeMS(200);
                                ead.initialize(1, solver);
                                //ead.initialize(1, new AStarSolver());
                                IntersectionData intersection1 = new IntersectionData(); // Id 9709
                                intersection1.setMap(mock(MapMessage.class));
                                intersection1.setRoughDist((int)(dist1*100.0));
                                intersection1.setDtsb(dist1);
                                intersection1.setCurrentPhase(phase1);
                                intersection1.setTimeToNextPhase((double)i);
                                intersection1.setStopBoxWidth(32.90);
                                intersection1.setIntersectionId(9709);
                                intersection1.setGeometry(new IntersectionGeometry(40, 100));
                                IntersectionData intersection2 = new IntersectionData(); // Id 9945
                                intersection2.setMap(mock(MapMessage.class));
                                intersection2.setRoughDist((int)(dist2*100.0));
                                intersection2.setDtsb(dist2);
                                intersection2.setCurrentPhase(phase2);
                                intersection2.setTimeToNextPhase((double)j);
                                intersection2.setStopBoxWidth(35.18);
                                intersection2.setIntersectionId(9945);
                                intersection2.setGeometry(new IntersectionGeometry(40, 100));
                                List<IntersectionData> intersections = Arrays.asList(intersection1, intersection2);
                                long startTime = System.currentTimeMillis();
                                try {
                                    List<Node> res = ead.plan(0, 11.176, intersections, 0, 0);
                                    totalPlanningTime += System.currentTimeMillis() - startTime;
                                    //totalIterationCount += ANAStarSolver.iterationCount;
                                    //System.out.println("A* Planning for two intersections takes " + (System.currentTimeMillis() - startTime) + " ms to finish");
                                    // for(Node n : res) {
                                    //     System.out.println(n.toString());
                                    // }
                                } catch(Exception e) {
                                    System.out.println("FAILED: DTSB1: " + dist1 + " DTSB2: " + dist2 + " Phase1: " + phase1 + " Phase2: " + phase2 + " timeToNext1: " + i + " timeToNext2: " + j);
                                    System.out.println("randomPredictionStartLoc: " + randomPredictionStartLoc + " randomPredictionStartTime: " + randomPredictionStartTime + " randomPredictionSpeed: " + randomPredictionSpeed);
                                    failureCount++;
                                    //e.printStackTrace();
                                    //assertTrue(false); // indicate the failure
                                }
                                totalCount++;
                            }
                        }
                        
                        phase2 = phase2.next();
                    }
                    
                    phase1 = phase1.next();
                }
                System.out.println("Completed: " + totalCount + " plans");
            }
        }
        System.out.println("\n\n ////// Test Complete //////");
        System.out.println("Total Plans: " + totalCount);
        System.out.println("Failed Plans: " + failureCount);
        System.out.println("Total ANA Iterations: " + totalIterationCount);
        System.out.println("Average Iterations Count: " + (totalIterationCount / (totalCount - failureCount))); 
        System.out.println("Percent Failed: " + ((double)failureCount / (double)totalCount));
        System.out.println("Average Planning Time ms: " + (totalPlanningTime / (totalCount - failureCount))); 
        System.out.println("\n\n");

        assertTrue(0 == failureCount);// Test fails if any plans fail
    }
    
    @Test
    public void planSingleIntersectionWithNCV() {
        // FAILED: DTSB1: 50.0 DTSB2: 130.0 Phase1: RED Phase2: RED timeToNext1: 11.0 timeToNext2: 10.0
        when(mockConfig.getDoubleDefaultValue("ead.coarse_time_inc", 5.0)).thenReturn(2.0);
        when(mockConfig.getDoubleDefaultValue("ead.coarse_speed_inc", 3.0)).thenReturn(2.0);
        when(mockConfig.getDoubleDefaultValue("ead.fine_time_inc", 2.0)).thenReturn(2.0);
        when(mockConfig.getDoubleDefaultValue("ead.fine_speed_inc", 1.0)).thenReturn(1.0);
        when(mockConfig.getDoubleDefaultValue("ead.acceptableStopDistance", 6.0)).thenReturn(6.0);
        ((MockCollisionChecker) mockCC).setPredictedTrajectory(new Node(20, 2, 1), new Node(26, 8, 1));
        long startTime = System.currentTimeMillis();
        EadAStar ead = new EadAStar(mockCC);
        ead.initialize(1, new AStarSolver());
        IntersectionData intersection1 = new IntersectionData(); // Id 9945
        intersection1.setMap(mock(MapMessage.class, Mockito.withSettings().stubOnly()));
        intersection1.setRoughDist(8423); 
        intersection1.setDtsb(84.23);
        intersection1.setCurrentPhase(SignalPhase.GREEN);
        intersection1.setTimeToNextPhase(26.99095117187494);
        intersection1.setStopBoxWidth(35.18);
        intersection1.setIntersectionId(9945);
        intersection1.setGeometry(new IntersectionGeometry(40, 100));
        List<IntersectionData> intersections = Arrays.asList(intersection1);
        try {
            List<Node> res = ead.plan(1.935252945217594, 11.176, intersections, 0, 0);
            // If ther is no prediction, the plan is:
//            Node{distance=       0, time=     0, speed=   2}
//            Node{distance=       6, time=     2, speed=   4}
//            Node{distance=      17, time=     4, speed=   7}
//            Node{distance=      34, time=     6, speed=  10}
//            Node{distance=      40, time=     7, speed=  11}
//            Node{distance=      62, time=     9, speed=  11}
//            Node{distance=      83, time=    11, speed=  10}
//            Node{distance=     104, time=    13, speed=  11}
//            Node{distance=     125, time=    15, speed=  10}
            System.out.println("A* Planning for one intersections takes " + (System.currentTimeMillis() - startTime) + " ms to finish");
            for(Node n : res) {
                System.out.println(n.toString());
            }
        } catch(Exception e) {
            e.printStackTrace();
            assertTrue(false); // indicate the failure
        }
    }
    
    @Test
    public void planTwoIntersectionsWithNCV() {
        when(mockConfig.getDoubleDefaultValue("ead.coarse_time_inc", 5.0)).thenReturn(2.0);
        when(mockConfig.getDoubleDefaultValue("ead.coarse_speed_inc", 3.0)).thenReturn(2.0);
        when(mockConfig.getDoubleDefaultValue("ead.fine_time_inc", 2.0)).thenReturn(2.0);
        when(mockConfig.getDoubleDefaultValue("ead.fine_speed_inc", 1.0)).thenReturn(1.0);
        when(mockConfig.getDoubleDefaultValue("ead.acceptableStopDistance", 6.0)).thenReturn(10.0);
        long startTime = System.currentTimeMillis();
        ((MockCollisionChecker) mockCC).setPredictedTrajectory(new Node(20, 2, 1), new Node(26, 8, 1));
        EadAStar ead = new EadAStar(mockCC);
        ead.initialize(1, new AStarSolver());
        IntersectionData intersection1 = new IntersectionData(); // Id 9709
        intersection1.setMap(mock(MapMessage.class, Mockito.withSettings().stubOnly()));
        intersection1.setRoughDist(2000);
        intersection1.setDtsb(20.00);
        intersection1.setCurrentPhase(SignalPhase.GREEN);
        intersection1.setTimeToNextPhase(15);
        intersection1.setStopBoxWidth(32.90);
        intersection1.setIntersectionId(9709);
        intersection1.setGeometry(new IntersectionGeometry(40, 100));
        IntersectionData intersection2 = new IntersectionData(); // Id 9945
        intersection2.setMap(mock(MapMessage.class, Mockito.withSettings().stubOnly()));
        intersection2.setRoughDist(10000);
        intersection2.setDtsb(100.00);
        intersection2.setCurrentPhase(SignalPhase.GREEN);
        intersection2.setTimeToNextPhase(15.0);
        intersection2.setStopBoxWidth(35.18);
        intersection2.setIntersectionId(9945);
        intersection2.setGeometry(new IntersectionGeometry(40, 100));
        List<IntersectionData> intersections = Arrays.asList(intersection1, intersection2);
        try {
            List<Node> res = ead.plan(1.9352529452127594, 11.176, intersections, 0, 0);
            System.out.println("A* Planning for two intersections takes " + (System.currentTimeMillis() - startTime) + " ms to finish");
            for(Node n : res) {
                System.out.println(n.toString());
            }
        } catch(Exception e) {
            e.printStackTrace();
            assertTrue(false); // indicate the failure
        }
    }


    @Test
    public void planTwoIntersectionWithComplexNCV() {

        int samples = 1;
        long totalTime = 0;
        for (int i =0; i < samples; i++) {

            // FAILED: DTSB1: 50.0 DTSB2: 130.0 Phase1: RED Phase2: RED timeToNext1: 11.0 timeToNext2: 10.0
            when(mockConfig.getDoubleDefaultValue("ead.coarse_time_inc", 5.0)).thenReturn(2.0);
            when(mockConfig.getDoubleDefaultValue("ead.coarse_speed_inc", 3.0)).thenReturn(2.0);
            when(mockConfig.getDoubleDefaultValue("ead.fine_time_inc", 2.0)).thenReturn(2.0);
            when(mockConfig.getDoubleDefaultValue("ead.fine_speed_inc", 1.0)).thenReturn(1.0);
            when(mockConfig.getDoubleDefaultValue("ead.acceptableStopDistance", 6.0)).thenReturn(6.0);


    //// Start setup of collision checker
            IMotionInterpolator planInterpolator = new PlanInterpolator();
            IMotionPredictorModelFactory motionPredictorFactory = new DefaultMotionPredictorFactory(mock(IGlidepathAppConfig.class, Mockito.withSettings().stubOnly()));
        
            PluginServiceLocator        psl = mock(PluginServiceLocator.class, Mockito.withSettings().stubOnly());
            ParameterSource             ps = mock(ParameterSource.class, Mockito.withSettings().stubOnly());
            RouteService                rs = mock(RouteService.class, Mockito.withSettings().stubOnly());
            ITimeProvider               tp = mock(ITimeProvider.class, Mockito.withSettings().stubOnly());
            IMobilityTimeProvider mobilityTimeProvider = mock(IMobilityTimeProvider.class, Mockito.withSettings().stubOnly());
            ArbitratorService           as = mock(ArbitratorService.class, Mockito.withSettings().stubOnly());
            Route                       route = mock(Route.class, Mockito.withSettings().stubOnly());
            RouteSegment mockSegment = mock(RouteSegment.class, Mockito.withSettings().stubOnly());
            List<RouteSegment> routeSegments = new ArrayList<>(Collections.nCopies(200, mockSegment)); // Create route full of the mocked segment
            when(route.getSegments()).thenReturn(routeSegments);
            when(mockSegment.determinePrimaryLane(anyDouble())).thenReturn(0);
        
            // Create conflict manager using default guidance settings
        
            // There may be a bug in ConflictManager or hash map because setting cell size for time to 3 makes the planning fail
            ConflictManager cm = new ConflictManager(new NSpatialHashMapFactory(new double[] {20.0, 15.0, 2.0}), 5.0, 1.2, 0.1, // USE THESE VALUES
            0.0, -0.25, 0.0, mobilityTimeProvider);
        
            cm.setRoute(route);
        
            when(psl.getParameterSource()).thenReturn(ps);
            when(psl.getRouteService()).thenReturn(rs);
            when(psl.getTimeProvider()).thenReturn(tp);
            when(psl.getConflictDetector()).thenReturn(cm);
            when(psl.getArbitratorService()).thenReturn(as);
        


            when(ps.getString("~ead/NCVHandling/objectMotionPredictorModel")).thenReturn("SIMPLE_LINEAR_REGRESSION");
            when(ps.getInteger("~ead/NCVHandling/collision/maxObjectHistoricalDataAge")).thenReturn(6000);
            when(ps.getDouble("~ead/NCVHandling/collision/distanceStep")).thenReturn(2.5);
            when(ps.getDouble("~ead/NCVHandling/collision/timeDuration")).thenReturn(3.0);
            when(ps.getDouble("~ead/NCVHandling/collision/replanPeriod")).thenReturn(1.5);
            when(ps.getDouble("~ead/NCVHandling/collision/downtrackBuffer")).thenReturn(3.0);
            when(ps.getDouble("~ead/NCVHandling/collision/crosstrackBuffer")).thenReturn(2.0);
            when(ps.getDouble("vehicle_length")).thenReturn(4.8768);
            when(ps.getDouble("vehicle_width")).thenReturn(2.1336);
            when(ps.getDouble("~ead/NCVHandling/collision/timeMargin")).thenReturn(0.2);
            when(ps.getDouble("~ead/NCVHandling/collision/longitudinalBias")).thenReturn(0.0);
            when(ps.getDouble("~ead/NCVHandling/collision/lateralBias")).thenReturn(0.0);
            when(ps.getDouble("~ead/NCVHandling/collision/temporalBias")).thenReturn(0.0);
            when(ps.getDouble("~ead/NCVHandling/collision/cell_downtrack_size")).thenReturn(20.0);
            when(ps.getDouble("~ead/NCVHandling/collision/cell_crosstrack_size")).thenReturn(15.0);
            when(ps.getDouble("~ead/NCVHandling/collision/cell_time_size")).thenReturn(2.0);

            ObjectCollisionChecker occ = new ObjectCollisionChecker(psl, motionPredictorFactory, planInterpolator, mockReplanHandle);

            //// End setup of collision checker

            //// Set starting state

            // Return the current lane id as 0 with crosstrack 0.0
            when(rs.getCurrentCrosstrackDistance()).thenReturn(0.0);
            when(rs.getCurrentRouteSegment()).thenReturn(mockSegment);
            double currentDowntrack = 0.0;
            when(rs.getCurrentDowntrackDistance()).thenReturn(currentDowntrack);
            when(rs.isRouteDataAvailable()).thenReturn(true);

            long currentTime = 510L; // The current time is 0.51
            when(tp.getCurrentTimeMillis()).thenReturn(currentTime);
            when(mobilityTimeProvider.getCurrentTimeMillis()).thenReturn(currentTime);

            //// End set starting state




            // Add one obstacle in current lane
            RoadwayObstacle ro = newRoadwayObstacle(0, 20, 0.5, 2.0); // Object id 0 with stamp at 0.5
            ro.setPrimaryLane((byte)0);
            occ.updateObjects(Arrays.asList(ro));

            currentTime = 610L; // The current time is 0.61

            // Obstacle is sampled a second time
            RoadwayObstacle ro2 = newRoadwayObstacle(0, 20.2, 0.6, 2.0); // Object id 0 with stamp at 0.6
            ro.setPrimaryLane((byte)0);
            occ.updateObjects(Arrays.asList(ro2));


            long startTime = System.currentTimeMillis();
            PLAN_START_TIME = startTime;
            EadAStar ead = new EadAStar(occ);
            ead.initialize(1, new AStarSolver());
            IntersectionData intersection1 = new IntersectionData(); // Id 9709
            intersection1.setMap(mock(MapMessage.class, Mockito.withSettings().stubOnly()));
            intersection1.setRoughDist(2000);
            intersection1.setDtsb(50.00);
            intersection1.setCurrentPhase(SignalPhase.GREEN);
            intersection1.setTimeToNextPhase(15);
            intersection1.setStopBoxWidth(32.90);
            intersection1.setIntersectionId(9709);
            intersection1.setGeometry(new IntersectionGeometry(40, 100));
            IntersectionData intersection2 = new IntersectionData(); // Id 9945
            intersection2.setMap(mock(MapMessage.class, Mockito.withSettings().stubOnly()));
            intersection2.setRoughDist(10000);
            intersection2.setDtsb(130.00);
            intersection2.setCurrentPhase(SignalPhase.GREEN);
            intersection2.setTimeToNextPhase(15.0);
            intersection2.setStopBoxWidth(35.18);
            intersection2.setIntersectionId(9945);
            intersection2.setGeometry(new IntersectionGeometry(40, 100));
            List<IntersectionData> intersections = Arrays.asList(intersection1, intersection2);
            try {
               // System.out.println("Plan Start Time: " + (double)(currentTime + 10L) / 1000.0);
                List<Node> res = ead.plan(0.0, 11.176, intersections, (double)(currentTime + 10L) / 1000.0, currentDowntrack);
                //System.out.println("TimeSpentBuildingMaps: " + TimeUnit.NANOSECONDS.toMillis(ConflictManager.nanoSecBuilding));
                //System.out.println("TimeSpentCheckingMaps: " + TimeUnit.NANOSECONDS.toMillis(ConflictManager.nanoSecChecking));
                // If there is no prediction, the plan is:
    //            Node{distance=       0, time=     0, speed=   2}
    //            Node{distance=       6, time=     2, speed=   4}
    //            Node{distance=      17, time=     4, speed=   7}
    //            Node{distance=      34, time=     6, speed=  10}
    //            Node{distance=      40, time=     7, speed=  11}
    //            Node{distance=      62, time=     9, speed=  11}
    //            Node{distance=      83, time=    11, speed=  10}
    //            Node{distance=     104, time=    13, speed=  11}
    //            Node{distance=     125, time=    15, speed=  10}
                totalTime += System.currentTimeMillis() - startTime;
                System.out.println("A* Planning for one intersections takes " + (System.currentTimeMillis() - startTime) + " ms to finish");
                for(Node n : res) {
                    System.out.println(n.toString());
                }
            } catch(Exception e) {
                e.printStackTrace();
                assertTrue(false); // indicate the failure
            }
        }
        System.out.println("AverageTime: " + (totalTime / samples));
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
