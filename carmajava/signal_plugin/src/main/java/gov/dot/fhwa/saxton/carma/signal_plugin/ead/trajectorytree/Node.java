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

package gov.dot.fhwa.saxton.carma.signal_plugin.ead.trajectorytree;

import java.util.Objects;

/**
 * Node class which represents the vehicle state at a certain time
 * Must override the hashCode and equals functions to work with ITreeSolver
 * hashCode will only map nodes based on state not memory location
 * so new Node(1,1,1).hashCode() == new Node(1,1,1).hashCode() same goes for equals
 * State values are represented as 64bit long which are converted to doubles using the provided methods
 */
public class Node {
  private long distance;  //in internal units
  private long time;      //in internal units
  private long speed;     //in internal units

  private double distDouble; //m
  private double timeDouble; //sec
  private double speedDouble; //m/s

  private static final int DISTANCE_UNITS = -2; // distance is in units of 10^DISTANCE_UNITS meters
  private static final int TIME_UNITS = -1; //time in units of 10^TIME_UNITS sec
  private static final int SPEED_UNITS = DISTANCE_UNITS - TIME_UNITS; // speed is in units of 10^SPEED_UNITS (m/s)

  /**
   * Constructor that takes in real-world units
   * @param distance Distance in m
   * @param time Time in sec
   * @param speed Speed in m/s
   */
  public Node(double distance, double time, double speed) {
    //round the inputs to the nearest internal unit
    this.distance = (long)(distance*Math.pow(10.0, -(double)DISTANCE_UNITS) + 0.5);
    this.time     = (long)(time    *Math.pow(10.0, -(double)TIME_UNITS)     + 0.5);
    this.speed    = (long)(speed   *Math.pow(10.0, -(double)SPEED_UNITS)    + 0.5);
    //store the rounded values as doubles for faster future access
    distDouble = (double)(this.distance) * Math.pow(10.0, (double)DISTANCE_UNITS);
    timeDouble = (double)(this.time)     * Math.pow(10.0, (double)TIME_UNITS);
    speedDouble = (double)(this.speed)   * Math.pow(10.0, (double)SPEED_UNITS);

  }

  /**
   * Constructor for node that takes in internal units
   * @param distance The distance in units of distanceUnits
   * @param time The time in units of timeUnits
   * @param speed The speed in units of speedUnits
   */
  public Node(long distance, long time, long speed) {
    distDouble = (double)distance * Math.pow(10.0, (double)DISTANCE_UNITS);
    timeDouble = (double)time     * Math.pow(10.0, (double)TIME_UNITS);
    speedDouble = (double)speed   * Math.pow(10.0, (double)SPEED_UNITS);
    this.distance = distance;
    this.time = time;
    this.speed = speed;
  }

  /**
   * @return Distance in internal units
   */
  public long getDistance() {
    return distance;
  }

  /**
   * @return Time in internal units
   */
  public long getTime() {
    return time;
  }

  /**
   * @return Speed in internal units
   */
  public long getSpeed() {
    return speed;
  }

  /**
   * @return Distance in units of m
   */
  public double getDistanceAsDouble() {
    return distDouble;
  }

  /**
   * @return Time in units of sec
   */
  public double getTimeAsDouble() {
    return timeDouble;
  }

  /**
   * @return Speed in units of m/s
   */
  public double getSpeedAsDouble() {
    return speedDouble;
  }

  public static int getDistanceUnits() {
    return DISTANCE_UNITS;
  }

  public static int getTimeUnits() {
    return TIME_UNITS;
  }

  public static int getSpeedUnits() {
    return SPEED_UNITS;
  }

  @Override public int hashCode() {
    return Objects.hash(distance, speed, time);
  }

  @Override public boolean equals(Object obj) {
    if (this == obj) {
      return true;
    } else if (!(obj instanceof Node)) {
      return false;
    } else {
      Node n2 = (Node) obj;
      return this.distance == n2.distance && this.speed == n2.speed && this.time == n2.time;
    }
  }

  @Override public String toString() {
    return String.format("Node{distance=%8d, time=%6d, speed=%4d}", distance, time, speed);
  }
}
