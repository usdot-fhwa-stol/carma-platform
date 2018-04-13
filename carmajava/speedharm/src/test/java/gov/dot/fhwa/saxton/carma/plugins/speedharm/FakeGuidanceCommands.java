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

package gov.dot.fhwa.saxton.carma.plugins.speedharm;


import gov.dot.fhwa.saxton.carma.guidance.IGuidanceCommands;

public class FakeGuidanceCommands implements IGuidanceCommands {

    private double speedCmd_ = -1.0;
    private double accelCmd_ = -1.0;


    @Override
    public void setSpeedCommand(double speed, double accel) {
        speedCmd_ = speed;
        accelCmd_ = accel;
    }

    public double getSpeedCmd() {
        return speedCmd_;
    }

    public double getAccelCmd() {
        return accelCmd_;
    }

    @Override
    public void setSteeringCommand(double axleAngle, double lateralAccel, double yawRate) {
        // TODO Auto-generated method stub
        
    }
}
