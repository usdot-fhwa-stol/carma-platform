/*
 * Copyright (C) 2018-2019 LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

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
        return DEFAULT_SPEED - 2.509;
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

        //build in a delay to simulate the rest of the guidance thread executing
        try {
            Thread.sleep(99);
        }catch (InterruptedException e) {
        }

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
	public double getDistanceToFrontVehicle() {
		return 0;
	}


	@Override
	public double getFrontVehicleSpeed() {
		return 0;
	}


	@Override
	public int getCurrentLane() {
		return 0;
	}


    @Override
    public double getCrosstrackDistance() {
        // TODO Auto-generated method stub
        return 0;
    }


	@Override
	public double getMaxAccelLimit() {
		return ACCEL;
	}
}
