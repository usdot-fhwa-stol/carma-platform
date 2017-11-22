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

import cav_msgs.ExternalObjectList;
import cav_msgs.RobotEnabled;
import geometry_msgs.Twist;
import geometry_msgs.TwistStamped;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.IPubSubService;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.ISubscriber;
import gov.dot.fhwa.saxton.speedharm.api.objects.VehicleStatusUpdate.AutomatedControlStatus;

/**
 * Manages recieving data from the vehicle
 */
public class VehicleDataManager {
  protected IPubSubService pubSubService;
  protected AutomatedControlStatus automatedControl;
  protected double range;
  protected double heading;
  protected double latitude;
  protected double longitude;
  protected double rangeRate;
  protected double speed;
  protected double accel;

  // ROS Subscribers
  protected ISubscriber<RobotEnabled> robotStatusSubscriber;
  protected ISubscriber<ExternalObjectList> radarSubscriber;
  protected ISubscriber<NavSatFix> navSatSubscriber;
  protected ISubscriber<TwistStamped> twistSubscriber;

  public void init() {
    robotStatusSubscriber = pubSubService.getSubscriberForTopic("robot_enabled", RobotEnabled._TYPE);
    radarSubscriber

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