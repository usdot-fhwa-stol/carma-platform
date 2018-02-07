/*
 * Copyright (C) 2018 LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not * use this file except in compliance with the License. You may obtain a copy of
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

import java.util.Comparator;
import java.util.List;
import java.util.Optional;
import cav_msgs.ExternalObject;
import cav_msgs.ExternalObjectList;
import cav_msgs.HeadingStamped;
import cav_msgs.RobotEnabled;
import geometry_msgs.AccelStamped;
import geometry_msgs.TwistStamped;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.IPubSubService;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.ISubscriber;
import gov.dot.fhwa.saxton.speedharm.api.objects.VehicleStatusUpdate.AutomatedControlStatus;
import sensor_msgs.NavSatFix;

/**
 * Manages recieving data from the vehicle
 */
public class VehicleDataManager {
  protected IPubSubService pubSubService;
  protected volatile AutomatedControlStatus automatedControl;
  protected volatile double range;
  protected volatile double heading;
  protected volatile double latitude;
  protected volatile double longitude;
  protected volatile double rangeRate;
  protected volatile double speed;
  protected volatile double accel;
  protected volatile boolean maneuverRunning = false;

  // ROS Subscribers
  protected ISubscriber<RobotEnabled> robotStatusSubscriber;
  protected ISubscriber<ExternalObjectList> radarSubscriber;
  protected ISubscriber<sensor_msgs.NavSatFix> navSatSubscriber;
  protected ISubscriber<TwistStamped> twistSubscriber;
  protected ISubscriber<AccelStamped> accelSubscriber;
  protected ISubscriber<HeadingStamped> headingSubscriber;

  /**
   * Begin the subscriptions to vehicle data
   */
  public void init(IPubSubService pubSubService) {
    this.pubSubService = pubSubService;
    robotStatusSubscriber = pubSubService.getSubscriberForTopic("robot_enabled", RobotEnabled._TYPE);
    radarSubscriber = pubSubService.getSubscriberForTopic("objects", ExternalObjectList._TYPE);
    navSatSubscriber = pubSubService.getSubscriberForTopic("nav_sat_fix", NavSatFix._TYPE);
    twistSubscriber = pubSubService.getSubscriberForTopic("velocity", TwistStamped._TYPE);
    accelSubscriber = pubSubService.getSubscriberForTopic("acceleration", AccelStamped._TYPE);
    headingSubscriber = pubSubService.getSubscriberForTopic("heading", HeadingStamped._TYPE);

    robotStatusSubscriber.registerOnMessageCallback(msg -> {
      // TODO: Maybe somehow this is aware of the maneuver's execution status?
      if (msg.getRobotActive()) {
        if (maneuverRunning) {
          automatedControl = AutomatedControlStatus.ENGAGED;
        } else {
          automatedControl = AutomatedControlStatus.ENGAGED_BUT_IGNORING;
        }
      } else {
        automatedControl = AutomatedControlStatus.DISENGAGED;
      }
    });

    radarSubscriber.registerOnMessageCallback(msg -> {
      Optional<ExternalObject> closest = msg.getObjects().stream()
          .sorted((ExternalObject obj1, ExternalObject obj2) -> {
            return Double.compare(obj2.getPose().getPose().getPosition().getX(),
                obj1.getPose().getPose().getPosition().getX());
          }).findFirst();

      closest.ifPresent((obj) -> {
        range = obj.getPose().getPose().getPosition().getX();
        rangeRate = obj.getRangeRate();
      });
    });

    navSatSubscriber.registerOnMessageCallback(msg -> {
      latitude = msg.getLatitude();
      longitude = msg.getLongitude();
    });

    twistSubscriber.registerOnMessageCallback(msg -> speed = msg.getTwist().getLinear().getX());
    accelSubscriber.registerOnMessageCallback(msg -> accel = msg.getAccel().getLinear().getX());
    headingSubscriber.registerOnMessageCallback(msg -> heading = msg.getHeading());
  }

  public AutomatedControlStatus getAutomatedControl() {
    return automatedControl;
  }

  public void setManeuverRunning(boolean value) {
    maneuverRunning = value;
    if (maneuverRunning && automatedControl == AutomatedControlStatus.ENGAGED_BUT_IGNORING) {
      automatedControl = AutomatedControlStatus.ENGAGED;
    } else if (!maneuverRunning && automatedControl == AutomatedControlStatus.ENGAGED) {
      automatedControl = AutomatedControlStatus.ENGAGED_BUT_IGNORING;
    }
  }

  /**
   * Get the distance to the nearest radar object on the long range radars
   */
  public double getRange() {
    return range;
  }

  /**
   * Get the heading of the vehicle in degrees
   */
  public double getHeading() {
    return heading;
  }

  /**
   * Get the latitude of the vehicle in degrees
   */
  public double getLatitude() {
    return latitude;
  }

  /**
   * Get the longitude of the vehicle in degrees
   */
  public double getLongitude() {
    return longitude;
  }

  /**
   * Get the relative speed of the nearest radar object in meters per second
   */
  public double getRangeRate() {
    return rangeRate;
  }

  /**
   * Get the current speed of the vehicle in m/s
   */
  public double getSpeed() {
    return speed;
  }

  /**
   * Get the current acceleration of the vehicle in m/s/s
   */
  public double getAccel() {
    return accel;
  }
}