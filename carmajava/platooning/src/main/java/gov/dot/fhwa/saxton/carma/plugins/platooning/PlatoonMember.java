/*
 * Copyright (C) 2017 LEIDOS.
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

public class PlatoonMember {
    
    private int memberId;
    private String staticId;
    private double commandSpeed;
    private double vehicleSpeed;
    private double vehiclePosition;

    public PlatoonMember(int memberId, String staticId, double commandSpeed, double vehicleSpeed, double vehiclePosition) {
        // Member ID indicates where the vehicle is in the platoon. When its ID is 0, it means it is the leader.
        this.memberId = memberId;
        this.staticId = staticId;
        this.commandSpeed = commandSpeed;
        this.vehicleSpeed = vehicleSpeed;
        this.vehiclePosition = vehiclePosition;
    }

    protected int getMemberId() {
        return memberId;
    }

    protected void setMemberId(int memberId) {
        this.memberId = memberId;
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
}
