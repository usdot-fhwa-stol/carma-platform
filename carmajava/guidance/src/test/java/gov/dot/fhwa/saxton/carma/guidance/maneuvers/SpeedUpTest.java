package gov.dot.fhwa.saxton.carma.guidance.maneuvers;

import static junit.framework.Assert.assertEquals;

public class SpeedUpTest {
    private FakeManeuverInputs   inputs_;
    private FakeGuidanceCommands commands_;


    @Before
    void setup() {
        inputs_ = new FakeManeuverInputs();
        commands_ = new FakeGuidanceCommands();
    }

    @Test
    void testSpeedUpNominal() {
        //specify the start & end speeds
        double targetSpeed = inputs_.getTargetSpeed();
        double maxAccel = inputs_.getAccel();

        //plan the maneuver
        SpeedUp mvr = new SpeedUp();
        double startSpeed = inputs_.getSlowSpeed();
        mvr.setSpeeds(startSpeed, targetSpeed);
        mvr.setMaxAccel(maxAccel);

        double startDist = inputs_.getStartDist();
        mvr.plan(inputs_, commands_, startDist);
        double endDist = mvr.getEndDistance();

        //compute expected distance required to perform the maneuver
        double deltaV = targetSpeed - startSpeed;
        double mvrLength = startSpeed*deltaV/maxAccel + 0.5*deltaV*deltaV/maxAccel;
        assertEquals(startDist + mvrLength, endDist, 0.1);

        //execute the maneuver for several time steps
        inputs_.setTestCase("SpeedUpNominal");
        for (int i = 0;  i < 20;  ++i) {
            mvr.executeTimeStep();
            double speedCmd = commands_.getSpeedCmd();
            double accelCmd = commands_.getAccelCmd();
            assertEquals(inputs_.getAccel(), accelCmd, 0.001);

            double expectedSpeedCmd = Math.min(inputs_.getSlowSpeed() + (double)i * 0.1 * accelCmd, inputs_.getTargetSpeed());
            assertEquals(expectedSpeedCmd, speedCmd, 0.01);
        }
    }
}
