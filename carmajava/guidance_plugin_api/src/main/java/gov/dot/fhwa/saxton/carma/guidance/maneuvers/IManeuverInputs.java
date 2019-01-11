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

/**
 * Specifies the data input interface for a maneuver.  This keeps knowledge of the ROS network out of the remainder
 * of the package.
 */
public interface IManeuverInputs {

    /**
     * Provides the vehicle's current distance from the beginning of the planned route.
     * @return distance, m
     */
    double getDistanceFromRouteStart();

    /**
     * Provides the vehicle's current crosstrack distance on the route
     * @return distance, m
     */
    double getCrosstrackDistance();

    /**
     * Provides the vehicle's current speed.
     * @return current speed, m/s
     */
    double getCurrentSpeed();

    /**
     * Provides the vehicle's expected dynamic response lag time.
     * @return lag time, sec
     */
    double getResponseLag();

    /**
     * Provides the distance to the rear bumper of the nearest vehicle in the current lane
     * @return distance, m, {@link IAccStrategy.NO_FRONT_VEHICLE_DISTANCE} if no such vehicle
     */
    double getDistanceToFrontVehicle();

    /**
     * Provides the speed of the nearest vehicle in the current lane
     * @return speed, m/s, {@link IAccStrategy.NO_FRONT_VEHICLE_SPEED} if no such vehicle
     */
    double getFrontVehicleSpeed();

    /**
     * Get current vehicle lane index
     * @return the index of the vehicles current lane
     */
    int getCurrentLane();

    /**
     * Get the maximum acceleration achievable by the vehicle controller
     * @return the maximum acceleration in m/s^2
     */
    double getMaxAccelLimit();
}
