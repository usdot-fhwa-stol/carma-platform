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

package gov.dot.fhwa.saxton.carma.plugins.speedharm;

import gov.dot.fhwa.saxton.speedharm.api.objects.VehicleStatusUpdate.AutomatedControlStatus;

public class VehicleDataManager {
  protected AutomatedControlStatus automatedControl;
  protected double range;
  protected double heading;
  protected double latitude;
  protected double longitude;
  protected double rangeRate;
  protected double speed;
  protected double accel;

  public void init() {

  }

  public AutomatedControlStatus getAutomatedControl() {
    return automatedControl;
  }

  public double getRange() {
    return range;
  }

  public double getHeading() {
    return heading;
  }

  public double getLatitude() {
    return latitude;
  }

  public double getLongitude() {
    return longitude;
  }

  public double getRangeRate() {
    return rangeRate;
  }

  public double getSpeed() {
    return speed;
  }

  public double getAccel() {
    return accel;
  }
}