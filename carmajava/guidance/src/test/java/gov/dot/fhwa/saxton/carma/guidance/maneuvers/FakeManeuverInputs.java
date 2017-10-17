package gov.dot.fhwa.saxton.carma.guidance.maneuvers;

public class FakeManeuverInputs implements IManeuverInputs {

    private int             iDist_ = 0;
    private int             iSpeed_ = 0;
    private String          testCase_ = "";
    private final double    START_DIST = 6318.2;
    private final double    DEFAULT_SPEED = 14.33;

    public void setTestCase(String name) {
        testCase_ = name;
        iDist_ = 0;
        iSpeed_ = 0;
    }


    public double getStartDist() {
        return START_DIST;
    }

    @Override
    public double getDistanceFromRouteStart() {

        switch(testCase_) {
            case "SteadySpeedNominal-1":
                return START_DIST - 24.1;

            case "SteadySpeedNominal-2":
                return START_DIST + (double)(iDist_++) * 0.1 * DEFAULT_SPEED;

            default:
                return START_DIST;
        }
    }

    @Override
    public double getCurrentSpeed() {

        switch(testCase_) {

            default:
                return DEFAULT_SPEED;
        }
    }
}
