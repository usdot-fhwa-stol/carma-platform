package gov.dot.fhwa.saxton.carma.guidance.maneuvers;

import org.junit.Before;
import org.junit.Test;

import static junit.framework.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

public class SlowDownTest {
    private FakeManeuverInputs   inputs_;
    private FakeGuidanceCommands commands_;


    @Before
    public void setup() {
        inputs_ = new FakeManeuverInputs();
        commands_ = new FakeGuidanceCommands();
    }

    @Test
    public void testSlowDownNominal() {
        //specify the start & end speeds
        double targetSpeed = inputs_.getTargetSpeed();
        double maxAccel = inputs_.getAccel();

        //plan the maneuver
        SlowDown mvr = new SlowDown();
        double startSpeed = inputs_.getFastSpeed();
        mvr.setSpeeds(startSpeed, targetSpeed);
        mvr.setMaxAccel(maxAccel);

        double startDist = inputs_.getStartDist();
        mvr.plan(inputs_, commands_, startDist);
        double endDist = mvr.getEndDistance();

        //compute expected distance required to perform the maneuver
        double deltaV = targetSpeed - startSpeed;
        double mvrLength = -startSpeed*deltaV/maxAccel + 0.5*deltaV*deltaV/maxAccel + 0.2*targetSpeed;
        assertEquals(startDist + mvrLength, endDist, 0.1);

        //execute the maneuver for several time steps
        inputs_.setTestCase("SlowDownNominal");
        double prevCmd = 999.0;
        boolean done;
        int i = 0;
        do {
            done = mvr.executeTimeStep();
            double speedCmd = commands_.getSpeedCmd();
            double accelCmd = commands_.getAccelCmd();
            assertEquals(inputs_.getAccel(), accelCmd, 0.001);

            double expectedSpeedCmd = Math.max(inputs_.getFastSpeed() - (double)i * 0.1 * accelCmd, inputs_.getTargetSpeed());
            System.out.println("expected = " + expectedSpeedCmd + ", actual = " + speedCmd + ", prev = " + prevCmd);
            assertTrue(speedCmd <= prevCmd);
            assertEquals(expectedSpeedCmd, speedCmd, 0.02);
            prevCmd = speedCmd;
            ++i;
        } while(!done);
    }
}
