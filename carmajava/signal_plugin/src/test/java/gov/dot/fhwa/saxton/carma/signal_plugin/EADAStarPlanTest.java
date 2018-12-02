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
import gov.dot.fhwa.saxton.carma.signal_plugin.ead.trajectorytree.AStarSolver;
import gov.dot.fhwa.saxton.carma.signal_plugin.ead.trajectorytree.FinePathNeighbors;
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
        when(mockConfig.getMaximumSpeed(0.0)).thenReturn(25);
        when(mockConfig.getDoubleDefaultValue("crawlingSpeed", 5.0)).thenReturn(5.0);
        when(mockConfig.getDoubleDefaultValue("ead.timebuffer", 4.0)).thenReturn(timeBuffer);
        when(mockConfig.getDoubleDefaultValue("ead.response.lag", 1.9)).thenReturn(0.0); // Was 1.9
        //when(mockConfig.getProperty("ead.desiredCostModel")).thenReturn("DEFAULT");
        when(mockConfig.getProperty("ead.desiredCostModel")).thenReturn("MOVES_2010");
        when(mockConfig.getDoubleValue("ead.MOVES.rollingTermA")).thenReturn(0.22112); 
        when(mockConfig.getDoubleValue("ead.MOVES.rotatingTermB")).thenReturn(0.002838); 
        when(mockConfig.getDoubleValue("ead.MOVES.dragTermC")).thenReturn(0.000698); 
        when(mockConfig.getDoubleValue("ead.MOVES.vehicleMassInTons")).thenReturn(1.86686); 
        when(mockConfig.getDoubleValue("ead.MOVES.fixedMassFactor")).thenReturn(1.86686); 
        when(mockConfig.getProperty("ead.MOVES.baseRateTablePath")).thenReturn("/opt/carma/src/CARMAPlatform/carmajava/launch/params/BaseRateForPassengerTruck.csv");
        
        when(mockConfig.getDoubleValue("ead.MOVES.fuelNormalizationDenominator")).thenReturn(425000.0); 
        when(mockConfig.getDoubleValue("ead.MOVES.timeNormalizationDenominator")).thenReturn(2.0); 
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
        long startTime = System.currentTimeMillis();
        EadAStar ead = new EadAStar(mockCC);
        ead.initialize(1, new AStarSolver());
        IntersectionData intersection1 = new IntersectionData(); // Id 9945
        intersection1.map = mock(MapMessage.class, Mockito.withSettings().stubOnly());
        intersection1.roughDist = 1581; 
        intersection1.dtsb = 50;
        intersection1.currentPhase = SignalPhase.GREEN;
        intersection1.timeToNextPhase = 8.631097656249949;
        intersection1.stopBoxWidth = 32.90;
        intersection1.intersectionId = 9709;
        intersection1.geometry = new IntersectionGeometry(40, 100);
        List<IntersectionData> intersections = Arrays.asList(intersection1);
        try {
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
        // FAILED: DTSB1: 50.0 DTSB2: 130.0 Phase1: RED Phase2: RED timeToNext1: 11.0 timeToNext2: 10.0
        when(mockConfig.getDoubleDefaultValue("ead.coarse_time_inc", 5.0)).thenReturn(2.0);
        when(mockConfig.getDoubleDefaultValue("ead.coarse_speed_inc", 3.0)).thenReturn(2.0);
        when(mockConfig.getDoubleDefaultValue("ead.fine_time_inc", 2.0)).thenReturn(2.0);
        when(mockConfig.getDoubleDefaultValue("ead.fine_speed_inc", 1.0)).thenReturn(1.0);
        when(mockConfig.getDoubleDefaultValue("ead.acceptableStopDistance", 6.0)).thenReturn(6.0);
        long startTime = System.currentTimeMillis();
        EadAStar ead = new EadAStar(mockCC);
        ead.initialize(1, new AStarSolver());
        IntersectionData intersection1 = new IntersectionData(); // Id 9709
        intersection1.map = mock(MapMessage.class, Mockito.withSettings().stubOnly());
        intersection1.roughDist = 5700;
        intersection1.dtsb = 40.49;
        intersection1.currentPhase = SignalPhase.RED;
        intersection1.timeToNextPhase = 29.82590234374993;
        intersection1.stopBoxWidth = 32.90;
        intersection1.intersectionId = 9709;
        intersection1.geometry = new IntersectionGeometry(40, 100);
        IntersectionData intersection2 = new IntersectionData(); // Id 9945
        intersection2.map = mock(MapMessage.class, Mockito.withSettings().stubOnly());
        intersection2.roughDist = 22394;
        intersection2.dtsb = 211.48;
        intersection2.currentPhase = SignalPhase.RED;
        intersection2.timeToNextPhase = 4.3409023437498035;
        intersection2.stopBoxWidth = 35.18;
        intersection2.intersectionId = 9945;
        intersection2.geometry = new IntersectionGeometry(40, 100);
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
    public void planManyCases() {
        when(mockConfig.getDoubleDefaultValue("ead.coarse_time_inc", 5.0)).thenReturn(2.0);
        when(mockConfig.getDoubleDefaultValue("ead.coarse_speed_inc", 3.0)).thenReturn(2.0);
        when(mockConfig.getDoubleDefaultValue("ead.fine_time_inc", 2.0)).thenReturn(2.0);
        when(mockConfig.getDoubleDefaultValue("ead.fine_speed_inc", 1.0)).thenReturn(1.0);
        when(mockConfig.getDoubleDefaultValue("ead.acceptableStopDistance", 6.0)).thenReturn(6.0);
        int totalCount = 0;
        int failureCount = 0;
        double totalPlanningTime = 0.0;
        SignalPhase phase1 = SignalPhase.GREEN;
        SignalPhase phase2 = SignalPhase.GREEN;
        EadAStar ead;
        AStarSolver solver;
        for (double dist1 = 20; dist1 < 100; dist1 += 30.0) {
            for (double dist2 = dist1 + 80; dist2 < 250; dist2 += 30.0) {
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
                                long startTime = System.currentTimeMillis();
                                //The following code is for NCV handling
                                int randomPredictionStartLoc = ThreadLocalRandom.current().nextInt((int)dist1, (int)dist2 + 1);
                                int randomPredictionStartTime = 0;
                                int randomPredictionSpeed = ThreadLocalRandom.current().nextInt(1, 11);
                                int predictionPeriod = 5;
                                ((MockCollisionChecker) mockCC).setPredictedTrajectory(
                                        new Node(randomPredictionStartLoc, randomPredictionStartTime, randomPredictionSpeed),
                                        new Node(randomPredictionStartLoc + predictionPeriod * randomPredictionSpeed, randomPredictionStartTime + predictionPeriod, randomPredictionSpeed));
                                ead = new EadAStar(mockCC);
                                solver = new AStarSolver();
                                ead.initialize(1, solver);
                                IntersectionData intersection1 = new IntersectionData(); // Id 9709
                                intersection1.map = mock(MapMessage.class);
                                intersection1.roughDist = (int)(dist1*100.0);
                                intersection1.dtsb = dist1;
                                intersection1.currentPhase = phase1;
                                intersection1.timeToNextPhase = (double)i;
                                intersection1.stopBoxWidth = 32.90;
                                intersection1.intersectionId = 9709;
                                intersection1.geometry = new IntersectionGeometry(40, 100);
                                IntersectionData intersection2 = new IntersectionData(); // Id 9945
                                intersection2.map = mock(MapMessage.class);
                                intersection2.roughDist = (int)(dist2*100.0);
                                intersection2.dtsb = dist2;
                                intersection2.currentPhase = phase2;
                                intersection2.timeToNextPhase = (double)j;
                                intersection2.stopBoxWidth = 35.18;
                                intersection2.intersectionId = 9945;
                                intersection2.geometry = new IntersectionGeometry(40, 100);
                                List<IntersectionData> intersections = Arrays.asList(intersection1, intersection2);
                                try {
                                    List<Node> res = ead.plan(0, 11.176, intersections, 0, 0);
                                    totalPlanningTime += System.currentTimeMillis() - startTime;
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
        System.out.println("Percent Failed: " + ((double)failureCount / (double)totalCount));
        System.out.println("Average Planning Time ms: " + (totalPlanningTime / (double)(totalCount - failureCount))); 
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
        intersection1.map = mock(MapMessage.class, Mockito.withSettings().stubOnly());
        intersection1.roughDist = 8423; 
        intersection1.dtsb = 84.23;
        intersection1.currentPhase = SignalPhase.GREEN;
        intersection1.timeToNextPhase = 26.99095117187494;
        intersection1.stopBoxWidth = 35.18;
        intersection1.intersectionId = 9945;
        intersection1.geometry = new IntersectionGeometry(40, 100);
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
        intersection1.map = mock(MapMessage.class, Mockito.withSettings().stubOnly());
        intersection1.roughDist = 2000;
        intersection1.dtsb = 20.00;
        intersection1.currentPhase = SignalPhase.GREEN;
        intersection1.timeToNextPhase = 15;
        intersection1.stopBoxWidth = 32.90;
        intersection1.intersectionId = 9709;
        intersection1.geometry = new IntersectionGeometry(40, 100);
        IntersectionData intersection2 = new IntersectionData(); // Id 9945
        intersection2.map = mock(MapMessage.class, Mockito.withSettings().stubOnly());
        intersection2.roughDist = 10000;
        intersection2.dtsb = 100.00;
        intersection2.currentPhase = SignalPhase.GREEN;
        intersection2.timeToNextPhase = 15.0;
        intersection2.stopBoxWidth = 35.18;
        intersection2.intersectionId = 9945;
        intersection2.geometry = new IntersectionGeometry(40, 100);
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
    public void planSingleIntersectionWithComplexNCV() {

        int samples = 25;
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
            when(ps.getInteger("~ead/NCVHandling/collision/maxObjectHistoricalDataAge")).thenReturn(3000);
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
            RoadwayObstacle ro = newRoadwayObstacle(0, 10, 0.5, 5.0); // Object id 0 with stamp at 0.5
            ro.setPrimaryLane((byte)0);
            occ.updateObjects(Arrays.asList(ro));

            currentTime = 610L; // The current time is 0.61

            // Obstacle is sampled a second time
            RoadwayObstacle ro2 = newRoadwayObstacle(0, 10.5, 0.6, 5.0); // Object id 0 with stamp at 0.6
            ro.setPrimaryLane((byte)0);
            occ.updateObjects(Arrays.asList(ro2));



            long startTime = System.currentTimeMillis();
            PLAN_START_TIME = startTime;
            EadAStar ead = new EadAStar(occ);
            ead.initialize(1, new AStarSolver());
            IntersectionData intersection1 = new IntersectionData(); // Id 9945
            intersection1.map = mock(MapMessage.class, Mockito.withSettings().stubOnly());
            intersection1.roughDist = 8423; 
            intersection1.dtsb = 84.23;
            intersection1.currentPhase = SignalPhase.GREEN;
            intersection1.timeToNextPhase = 26.99095117187494;
            intersection1.stopBoxWidth = 35.18;
            intersection1.intersectionId = 9945;
            intersection1.geometry = new IntersectionGeometry(40, 100);
            List<IntersectionData> intersections = Arrays.asList(intersection1);
            try {
               // System.out.println("Plan Start Time: " + (double)(currentTime + 10L) / 1000.0);
                List<Node> res = ead.plan(1.935252945217594, 11.176, intersections, (double)(currentTime + 10L) / 1000.0, 0);
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
                // System.out.println("A* Planning for one intersections takes " + (System.currentTimeMillis() - startTime) + " ms to finish");
                // for(Node n : res) {
                //     System.out.println(n.toString());
                // }
            } catch(Exception e) {
                e.printStackTrace();
                assertTrue(false); // indicate the failure
            }
        }
        System.out.println("AverageTime: " + (totalTime / samples));
    }


    @Test
    public void planSingleIntersectionWithComplexNCVTemp() {

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
            double currentDowntrack = 178.56449469994024;
            when(rs.getCurrentDowntrackDistance()).thenReturn(currentDowntrack);
            when(rs.isRouteDataAvailable()).thenReturn(true);

            long currentTime = 1542746461468L; // The current time is 0.51
            when(tp.getCurrentTimeMillis()).thenReturn(currentTime);
            when(mobilityTimeProvider.getCurrentTimeMillis()).thenReturn(currentTime);

            //// End set starting state
            List<RoutePointStamped> prediction = new ArrayList<>();
            prediction.add(new RoutePointStamped(181.9754265229875, 0.0, 1.542746462487E9));
            prediction.add(new RoutePointStamped(184.4754265229875, 0.0, 1.542746462829349E9));
            prediction.add(new RoutePointStamped(186.9754265229875, 0.0, 1.542746463171698E9));
            prediction.add(new RoutePointStamped(189.4754265229875, 0.0, 1.5427464635140471E9));
            prediction.add(new RoutePointStamped(191.9754265229875, 0.0, 1.5427464638563962E9));
            prediction.add(new RoutePointStamped(194.4754265229875, 0.0, 1.5427464641987453E9));
            prediction.add(new RoutePointStamped(196.9754265229875, 0.0, 1.5427464645410943E9));
            prediction.add(new RoutePointStamped(199.4754265229875, 0.0, 1.5427464648834434E9));
            prediction.add(new RoutePointStamped(201.9754265229875, 0.0, 1.5427464652257924E9));
            prediction.add(new RoutePointStamped(204.4754265229875, 0.0, 1.5427464655681415E9));
            prediction.add(new RoutePointStamped(206.9754265229875, 0.0, 1.5427464659104905E9));
            prediction.add(new RoutePointStamped(209.4754265229875, 0.0, 1.5427464662528396E9));
            prediction.add(new RoutePointStamped(211.9754265229875, 0.0, 1.5427464665951886E9));
            prediction.add(new RoutePointStamped(214.4754265229875, 0.0, 1.5427464669375377E9));
            prediction.add(new RoutePointStamped(216.9754265229875, 0.0, 1.5427464672798867E9));
            prediction.add(new RoutePointStamped(219.4754265229875, 0.0, 1.5427464676222358E9));
            prediction.add(new RoutePointStamped(221.9754265229875, 0.0, 1.5427464679645848E9));
            prediction.add(new RoutePointStamped(224.4754265229875, 0.0, 1.5427464683069339E9));
            prediction.add(new RoutePointStamped(226.9754265229875, 0.0, 1.542746468649283E9));

            occ.trackedLaneObjectsPredictions.put(0, prediction);



            long startTime = System.currentTimeMillis();
            PLAN_START_TIME = startTime;
            EadAStar ead = new EadAStar(occ);
            ead.initialize(1, new AStarSolver());
            IntersectionData intersection1 = new IntersectionData(); // Id 9945
            intersection1.map = mock(MapMessage.class, Mockito.withSettings().stubOnly());
            intersection1.roughDist = 1581; 
            intersection1.dtsb = -1.0;
            intersection1.currentPhase = SignalPhase.GREEN;
            intersection1.timeToNextPhase = 13.631097656249949;
            intersection1.stopBoxWidth = 32.90;
            intersection1.intersectionId = 9709;
            intersection1.geometry = new IntersectionGeometry(40, 100);
            List<IntersectionData> intersections = Arrays.asList(intersection1);
            try {
               // System.out.println("Plan Start Time: " + (double)(currentTime + 10L) / 1000.0);
                List<Node> res = ead.plan(10.228841293559354, 11.176, intersections, 1542746461.468, currentDowntrack);
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
                System.out.println("NumCollisions: " + FinePathNeighbors.numCollisions);
                e.printStackTrace();
                assertTrue(false); // indicate the failure
            }
        }
        System.out.println("AverageTime: " + (totalTime / samples));
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
            intersection1.map = mock(MapMessage.class, Mockito.withSettings().stubOnly());
            intersection1.roughDist = 2000;
            intersection1.dtsb = 50.00;
            intersection1.currentPhase = SignalPhase.GREEN;
            intersection1.timeToNextPhase = 15;
            intersection1.stopBoxWidth = 32.90;
            intersection1.intersectionId = 9709;
            intersection1.geometry = new IntersectionGeometry(40, 100);
            IntersectionData intersection2 = new IntersectionData(); // Id 9945
            intersection2.map = mock(MapMessage.class, Mockito.withSettings().stubOnly());
            intersection2.roughDist = 10000;
            intersection2.dtsb = 130.00;
            intersection2.currentPhase = SignalPhase.GREEN;
            intersection2.timeToNextPhase = 15.0;
            intersection2.stopBoxWidth = 35.18;
            intersection2.intersectionId = 9945;
            intersection2.geometry = new IntersectionGeometry(40, 100);
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
                System.out.println("NumCollisions: " + FinePathNeighbors.numCollisions);
            } catch(Exception e) {
                System.out.println("NumCollisions: " + FinePathNeighbors.numCollisions);
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
