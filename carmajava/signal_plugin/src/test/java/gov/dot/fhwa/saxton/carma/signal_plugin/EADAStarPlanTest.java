package gov.dot.fhwa.saxton.carma.signal_plugin;

import static org.junit.Assert.assertTrue;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

import java.util.Arrays;
import java.util.List;

import org.junit.Before;
import org.junit.Test;

import gov.dot.fhwa.saxton.carma.guidance.util.ILogger;
import gov.dot.fhwa.saxton.carma.guidance.util.ILoggerFactory;
import gov.dot.fhwa.saxton.carma.guidance.util.LoggerManager;
import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.IGlidepathAppConfig;
import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.SignalPhase;
import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.utils.GlidepathApplicationContext;
import gov.dot.fhwa.saxton.carma.signal_plugin.asd.IntersectionData;
import gov.dot.fhwa.saxton.carma.signal_plugin.asd.map.MapMessage;
import gov.dot.fhwa.saxton.carma.signal_plugin.ead.EadAStar;
import gov.dot.fhwa.saxton.carma.signal_plugin.ead.trajectorytree.AStarSolver;
import gov.dot.fhwa.saxton.carma.signal_plugin.ead.trajectorytree.Node;

public class EADAStarPlanTest {
    
    IGlidepathAppConfig mockConfig = mock(IGlidepathAppConfig.class);
    
    @Before
    public void setup() {
        ILoggerFactory mockFact = mock(ILoggerFactory.class);
        ILogger mockLogger = mock(ILogger.class);
        when(mockFact.createLoggerForClass(any())).thenReturn(mockLogger);
        LoggerManager.setLoggerFactory(mockFact);
        GlidepathApplicationContext.getInstance().setAppConfigOverride(mockConfig);
        when(mockConfig.getProperty("ead.desiredCostModel")).thenReturn("DEFAULT");
        when(mockConfig.getDoubleDefaultValue("defaultAccel", 2.0)).thenReturn(2.0);
        when(mockConfig.getMaximumSpeed(0.0)).thenReturn(25);
        when(mockConfig.getDoubleDefaultValue("crawlingSpeed", 5.0)).thenReturn(5.0);
        when(mockConfig.getDoubleDefaultValue("ead.timebuffer", 4.0)).thenReturn(4.0);
        when(mockConfig.getDoubleDefaultValue("ead.response.lag", 1.9)).thenReturn(1.9);
    }

    @Test
    public void planTwoIntersections() {
        when(mockConfig.getDoubleDefaultValue("ead.coarse_time_inc", 5.0)).thenReturn(4.0);
        when(mockConfig.getDoubleDefaultValue("ead.coarse_speed_inc", 3.0)).thenReturn(2.0);
        when(mockConfig.getDoubleDefaultValue("ead.fine_time_inc", 2.0)).thenReturn(3.0);
        when(mockConfig.getDoubleDefaultValue("ead.fine_speed_inc", 1.0)).thenReturn(3.0);
        when(mockConfig.getDoubleDefaultValue("ead.acceptableStopDistance", 6.0)).thenReturn(6.0);
        long startTime = System.currentTimeMillis();
        EadAStar ead = new EadAStar();
        ead.initialize(100, new AStarSolver());
        IntersectionData intersection1 = new IntersectionData();
        intersection1.map = mock(MapMessage.class);
        intersection1.roughDist = 7586;
        intersection1.dtsb = 75.86;
        intersection1.currentPhase = SignalPhase.GREEN;
        intersection1.timeToNextPhase = 5.0;
        IntersectionData intersection2 = new IntersectionData();
        intersection2.map = mock(MapMessage.class);
        intersection2.roughDist = 29298;
        intersection2.dtsb = 292.98;
        intersection2.currentPhase = SignalPhase.GREEN;
        intersection2.timeToNextPhase = 5.0;
        List<IntersectionData> intersections = Arrays.asList(intersection1, intersection2);
        try {
            List<Node> res = ead.plan(0, 10.3, intersections);
            System.out.println("A* Planning for two intersections takes " + (System.currentTimeMillis() - startTime) + " ms to finish");
            for(Node n : res) {
                System.out.println(n.toString());
            }
        } catch(Exception e) {
            e.printStackTrace();
            assertTrue(false); // indicate the failure
        }
    }
}
