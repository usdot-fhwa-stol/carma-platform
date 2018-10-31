/*
 * Copyright (C) 2018 LEIDOS.
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

package gov.dot.fhwa.saxton.carma.signal_plugin;

import gov.dot.fhwa.saxton.carma.guidance.maneuvers.IManeuverInputs;

/**
 * Wrapper for the base IManeuverInputs object used in the CARMA Platform
 * Allows alternative max acceleration and response lag values to be injected
 * Max acceleration will always be the smaller of either the platform wide max acceleration of the injected max accel. 
 */
public class TrafficSignalManeuverInputs implements IManeuverInputs {

    protected final IManeuverInputs platformInputs;
    protected final double maxAccel;
    protected final double responseLag;

    /**
     * Constructor 
     * 
     * @param platformInputs The base CARMA Platform Maneuver inputs object
     * @param responseLag The injected response lag time in seconds
     * @param maxAccel The injected max acceleration in m/s^2 
     * 
     * Max acceleration will always be the smaller of either the platform wide max acceleration of the injected max accel.
     */
    public TrafficSignalManeuverInputs(IManeuverInputs platformInputs, double responseLag, double maxAccel) {
        this.platformInputs = platformInputs;
        // Lag time is totally replaced
        this.responseLag = responseLag;
        // Use the smaller max acceleration value
        this.maxAccel = Math.min(platformInputs.getMaxAccelLimit(), maxAccel);
    }

    @Override
    public double getDistanceFromRouteStart() {
        return platformInputs.getDistanceFromRouteStart();
    }

    @Override
    public double getCurrentSpeed() {
        return platformInputs.getCurrentSpeed();
    }

    @Override
    public double getResponseLag() {
        return responseLag;
    }

    @Override
    public double getDistanceToFrontVehicle() {
        return platformInputs.getDistanceToFrontVehicle();
    }

    @Override
    public double getFrontVehicleSpeed() {
        return platformInputs.getFrontVehicleSpeed();
    }

    @Override
    public int getCurrentLane() {
        return platformInputs.getCurrentLane();
    }

    @Override
    public double getCrosstrackDistance() {
        return platformInputs.getCrosstrackDistance();
    }

	@Override
	public double getMaxAccelLimit() {
		return maxAccel;
	}
}
