package gov.dot.fhwa.saxton.carma.guidance.maneuvers;

public class FakeManeuverInputs implements IManeuverInputs {

    private int             iDist_ = 0;
    private int             iSpeed_ = 0;
    private String          testCase_ = "";
    private final double    START_DIST = 6318.2;
    private final double    DEFAULT_SPEED = 14.33;
    private final double    ACCEL = 1.5;

    public void setTestCase(String name) {
        testCase_ = name;
        iDist_ = 0;
        iSpeed_ = 0;
    }


    /**
     * These methods are for testing convenience only, to ensure that all tests line up to the same start location;
     * Not part of the production interface.
     */
    public double getStartDist() {
        return START_DIST;
    }

    public double getTargetSpeed() {
        return DEFAULT_SPEED;
    }

    public double getSlowSpeed() {
        return DEFAULT_SPEED - 3.1;
    }

    public double getFastSpeed() {
        return DEFAULT_SPEED + 4.4;
    }

    public double getAccel() {
        return ACCEL;
    }


    ////// implementing the production interface below here

    @Override
    public double getDistanceFromRouteStart() {
        double dist;

        switch(testCase_) {
            case "SteadySpeedNominal-1":
                dist = START_DIST - 24.1;
                break;

            case "SteadySpeedNominal-2":
                dist = START_DIST + (double)iDist_ * 0.1 * DEFAULT_SPEED;
                break;

            case "SpeedUpNominal":
                dist = START_DIST + (double)iDist_ * 0.1 * getCurrentSpeed();
                break;

            default:
                dist = START_DIST;
        }

        ++iDist_;
        ++iSpeed_;
        return dist;
    }

    @Override
    public double getCurrentSpeed() {
        double speed;

        switch(testCase_) {
            case "SpeedUpNominal":
                speed = getSlowSpeed() + 0.1*ACCEL*(double)iSpeed_;
                if (speed > getTargetSpeed()) speed = getTargetSpeed();
                break;

            case "SlowDownNominal":
                speed = getFastSpeed() - 0.1*ACCEL*(double)iSpeed_;
                if (speed < getTargetSpeed()) speed = getTargetSpeed();
                break;

            default:
                speed = DEFAULT_SPEED;
        }

        return speed;
    }

    @Override
    public double getResponseLag() {
        return 0.0;
    }

    @Override
    public int getTimeStep() {
        return 100;
    }
}
