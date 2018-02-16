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

package gov.dot.fhwa.saxton.carma.plugins.platooning;

/**
 * This class records the latest knowledge (status and constraints) for
 * each platoon member who is in front of the subject vehicle. 
 */
public class PlatoonMember {
    
    // Static ID is permanent ID for each vehicle
    private final String staticId;
    // Vehicle command speed in m/s after any possible ACC override
    private double commandSpeed;
    // The actual vehicle speed in m/s
    private double vehicleSpeed;
    // The vehicle position from Mobility message
    private double vehiclePosition;
    // The local timestamp when we received the status update of this member
    private long timestamp;

    public PlatoonMember(String staticId, double commandSpeed, double vehicleSpeed, double vehiclePosition, long timestamp) {
        this.staticId = staticId;
        this.commandSpeed = commandSpeed;
        this.vehicleSpeed = vehicleSpeed;
        this.vehiclePosition = vehiclePosition;
        this.timestamp = timestamp;
    }

    protected String getStaticId() {
        return staticId;
    }

    protected double getCommandSpeed() {
        return commandSpeed;
    }
    
    protected void setCommandSpeed(double commandSpeed) {
        this.commandSpeed = commandSpeed;
    }
    
    protected double getVehicleSpeed() {
        return vehicleSpeed;
    }
    
    protected void setVehicleSpeed(double vehicleSpeed) {
        this.vehicleSpeed = vehicleSpeed;
    }
    
    protected double getVehiclePosition() {
        return vehiclePosition;
    }
    
    protected void setVehiclePosition(double vehiclePosition) {
        this.vehiclePosition = vehiclePosition;
    }

    protected long getTimestamp() {
        return timestamp;
    }

    protected void setTimestamp(long timestamp) {
        this.timestamp = timestamp;
    }

    @Override
    public String toString() {
        return "PlatoonMember [staticId=" + staticId + ", commandSpeed=" + commandSpeed + ", vehicleSpeed="
                + vehicleSpeed + ", vehiclePosition=" + vehiclePosition + ", timestamp=" + timestamp + "]";
    }
}
