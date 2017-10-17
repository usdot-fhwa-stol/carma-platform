package gov.dot.fhwa.saxton.carma.guidance.maneuvers;

import com.google.common.annotations.VisibleForTesting;
import dot.gov.fhwa.saxton.carma.guidance.IGuidanceCommands;
import org.junit.*;

import static junit.framework.Assert.assertEquals;
import static junit.framework.Assert.assertTrue;

public class SteadySpeedTest {

    private IManeuverInputs     inputs_;
    private FakeGuidanceCommands commands_;


    @Before
    void setup() {
        inputs_ = new FakeManeuverInputs();
        commands_ = new FakeGuidanceCommands();
    }

    @Test
    public void testSteadySpeedNominal() {

        //specify the start & end speeds
        double targetSpeed = 14.33;
        double maxAccel = 1.5;

        //plan the maneuver
        SteadySpeed mvr = new SteadySpeed();
        mvr.setSpeeds(targetSpeed, targetSpeed); //yes, the same variable is repeated here
        mvr.setMaxAccel(maxAccel);

        double startDist = inputs_.getStartDist();
        mvr.plan(inputs_, commands_, startDist);
        double endDist = mvr.getEndDistance();
        assertEquals(startDist, endDist, 0.001);

        double newEndDist = startDist + 443.8;
        mvr.overrideEndDistance(newEndDist);
        endDist = mvr.getEndDistance();
        assertEquals(newEndDist, endDist, 0.001);

        assertEquals(startDist, mvr.getStartDistance(), 0.001);

        //execute the maneuver before it is scheduled
        inputs_.setTestCase("SteadySpeedNominal-1");
        try {
            mvr.executeTimeStep();
            assertTrue(false);
        }catch (IllegalStateException ise) {
            assertTrue(true);
        }

        //execute a couple time steps of the maneuver when it is supposed to happen
        inputs_.setTestCase("SteadySpeedNominal-2");
        try {
            for (int i = 0;  i < 3;  ++i) {
                mvr.executeTimeStep();
                double speedCmd = commands_.getSpeedCmd();
                double accelCmd = commands_.getAccelCmd();
                assertEquals(targetSpeed, speedCmd, 0.001);
                assertEquals(maxAccel, accelCmd, 0.001);
            }

        }catch(IllegalStateException ise) {
            assertTrue(false);
        }
    }


    @Test
    public void testSteadySpeedNoTarget() {

    }
}
