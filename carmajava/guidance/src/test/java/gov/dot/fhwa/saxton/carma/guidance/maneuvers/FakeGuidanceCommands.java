package gov.dot.fhwa.saxton.carma.guidance.maneuvers;


import gov.dot.fhwa.saxton.carma.guidance.IGuidanceCommands;

public class FakeGuidanceCommands implements IGuidanceCommands {

    private double speedCmd_ = -1.0;
    private double accelCmd_ = -1.0;


    @Override
    public void setCommand(double speed, double accel) {

    }

    public void setTestCase(String testCase) {

        switch (testCase) {

        }
    }

    public double getSpeedCmd() {
        return speedCmd_;
    }

    public double getAccelCmd() {
        return accelCmd_;
    }
}
