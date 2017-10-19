package gov.dot.fhwa.saxton.carma.guidance.maneuvers;

import org.junit.*;

import static junit.framework.Assert.assertEquals;
import static junit.framework.Assert.assertTrue;

public class SteadySpeedTest {

    private FakeManeuverInputs   inputs_;
    private FakeGuidanceCommands commands_;


    @Before
    public void setup() {
        inputs_ = new FakeManeuverInputs();
        commands_ = new FakeGuidanceCommands();
    }

    @Test
    public void testSteadySpeedNominal() {

        //specify the start & end speeds
        double targetSpeed = inputs_.getTargetSpeed();
        double maxAccel = inputs_.getAccel();

        //plan the maneuver
        SteadySpeed mvr = new SteadySpeed();
        mvr.setSpeeds(targetSpeed, targetSpeed); //yes, the same variable is repeated here for start & end speeds
        mvr.setMaxAccel(maxAccel);

        double startDist = inputs_.getStartDist();
        mvr.plan(inputs_, commands_, startDist);
        double endDist = mvr.getEndDistance();
        assertEquals(startDist, endDist, 0.001);

        double newEndDist = startDist + 143.8;
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
            boolean done;
            do {
                inputs_.getDistanceFromRouteStart(); //sleeps a while, making this test run fewer loops
                done = mvr.executeTimeStep();
                double speedCmd = commands_.getSpeedCmd();
                double accelCmd = commands_.getAccelCmd();
                assertEquals(targetSpeed, speedCmd, 0.001);
                assertEquals(0.5*maxAccel, accelCmd, 0.001);
            } while (!done);

        }catch(IllegalStateException ise) {
            assertTrue(false);
        }
    }


    @Test
    public void testSteadySpeedNoTarget() {

        SteadySpeed mvr = new SteadySpeed();
        double startDist = inputs_.getStartDist();
        try {
            mvr.plan(inputs_, commands_, startDist); //should throw exception
            double endDist = mvr.getEndDistance();
            assertEquals(startDist, endDist, 0.001);
            assertTrue(false);
        }catch (IllegalStateException ise) {
            assertTrue(true);
        }
    }
}
