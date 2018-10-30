package gov.dot.fhwa.saxton.carma.signal_plugin;

import static org.junit.Assert.assertTrue;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

import java.util.Arrays;
import java.util.List;

import org.junit.Before;
import org.junit.Test;
import org.mockito.Mockito;

import gov.dot.fhwa.saxton.carma.guidance.util.ILogger;
import gov.dot.fhwa.saxton.carma.guidance.util.ILoggerFactory;
import gov.dot.fhwa.saxton.carma.guidance.util.LoggerManager;
import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.IGlidepathAppConfig;
import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.SignalPhase;
import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.utils.GlidepathApplicationContext;
import gov.dot.fhwa.saxton.carma.signal_plugin.asd.IntersectionData;
import gov.dot.fhwa.saxton.carma.signal_plugin.asd.map.MapMessage;
import gov.dot.fhwa.saxton.carma.signal_plugin.ead.EadAStar;
import gov.dot.fhwa.saxton.carma.signal_plugin.ead.INodeCollisionChecker;
import gov.dot.fhwa.saxton.carma.signal_plugin.ead.IntersectionGeometry;
import gov.dot.fhwa.saxton.carma.signal_plugin.ead.trajectorytree.AStarSolver;
import gov.dot.fhwa.saxton.carma.signal_plugin.ead.trajectorytree.Node;

public class EADAStarPlanTest {
    
    IGlidepathAppConfig mockConfig = mock(IGlidepathAppConfig.class, Mockito.withSettings().stubOnly());
    INodeCollisionChecker mockCC = mock(INodeCollisionChecker.class, Mockito.withSettings().stubOnly());
    final double timeBuffer = 4.0;
    
    @Before
    public void setup() {
        ILoggerFactory mockFact = mock(ILoggerFactory.class, Mockito.withSettings().stubOnly());
        ILogger mockLogger = mock(ILogger.class, Mockito.withSettings().stubOnly());
        when(mockFact.createLoggerForClass(any())).thenReturn(mockLogger);
        LoggerManager.setLoggerFactory(mockFact);
        GlidepathApplicationContext.getInstance().setAppConfigOverride(mockConfig);
        when(mockConfig.getProperty("ead.desiredCostModel")).thenReturn("DEFAULT");
        when(mockConfig.getDoubleDefaultValue("defaultAccel", 2.0)).thenReturn(2.0);
        when(mockConfig.getMaximumSpeed(0.0)).thenReturn(25);
        when(mockConfig.getDoubleDefaultValue("crawlingSpeed", 5.0)).thenReturn(5.0);
        when(mockConfig.getDoubleDefaultValue("ead.timebuffer", 4.0)).thenReturn(timeBuffer);
        when(mockConfig.getDoubleDefaultValue("ead.response.lag", 1.9)).thenReturn(0.0); // Was 1.9
    }

    // These default phases will be used for phases which are not specified. Check NeighborBase.java to confirm.
    // protected final double                  DEFAULT_GREEN_DURATION = 27.0;  //sec
    // protected final double                  DEFAULT_YELLOW_DURATION = 3.0;  //sec
    // protected final double                  DEFAULT_RED_DURATION = 30.0;    //sec

    @Test
    public void planSingleIntersection() {
        // FAILED: DTSB1: 50.0 DTSB2: 130.0 Phase1: RED Phase2: RED timeToNext1: 11.0 timeToNext2: 10.0
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
        intersection1.roughDist = 8423; 
        intersection1.dtsb = 84.23;
        intersection1.currentPhase = SignalPhase.RED;
        intersection1.timeToNextPhase = 29.99095117187494;
        intersection1.stopBoxWidth = 35.18;
        intersection1.intersectionId = 9945;
        intersection1.geometry = new IntersectionGeometry(40, 100);
        List<IntersectionData> intersections = Arrays.asList(intersection1);
        try {
            List<Node> res = ead.plan(1.935252945217594, 11.176, intersections);
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
            List<Node> res = ead.plan(1.9352529452127594, 11.176, intersections);
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
                                    List<Node> res = ead.plan(0, 11.176, intersections);
                                    totalPlanningTime += System.currentTimeMillis() - startTime;
                                    //System.out.println("A* Planning for two intersections takes " + (System.currentTimeMillis() - startTime) + " ms to finish");
                                    // for(Node n : res) {
                                    //     System.out.println(n.toString());
                                    // }
                                } catch(Exception e) {
                                    System.out.println("FAILED: DTSB1: " + dist1 + " DTSB2: " + dist2 + " Phase1: " + phase1 + " Phase2: " + phase2 + " timeToNext1: " + i + " timeToNext2: " + j);
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
}
