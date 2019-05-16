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

package gov.dot.fhwa.saxton.carma.plugins.platooning;

/**
 * This class describes the latest knowledge for a platoon member.
 * This class will not be used to store the host vehicle status.
 */
public class PlatoonMember {
    
    // Static ID is permanent ID for each vehicle
    protected String staticId;
    // Current BSM Id for each CAV
    protected String bsmId;
    // Vehicle real time command speed in m/s
    protected double commandSpeed;
    // Actual vehicle speed in m/s
    protected double vehicleSpeed;
    // Vehicle current down track distance on the current route in m
    protected double vehiclePosition;
    // The local time stamp when the host vehicle update any informations of this member
    protected long   timestamp;

    public PlatoonMember(String staticId, String bsmId, double commandSpeed, double vehicleSpeed, double vehiclePosition, long timestamp) {
        this.staticId        = staticId;
        this.bsmId           = bsmId;
        this.commandSpeed    = commandSpeed;
        this.vehicleSpeed    = vehicleSpeed;
        this.vehiclePosition = vehiclePosition;
        this.timestamp       = timestamp;
    }

    @Override
    public String toString() {
        return "PlatoonMember [staticId=" + staticId + ", bsmId=" + bsmId + ", commandSpeed=" + commandSpeed
                + ", vehicleSpeed=" + vehicleSpeed + ", vehiclePosition=" + vehiclePosition + ", timestamp=" + timestamp + "]";
    }
    
}
