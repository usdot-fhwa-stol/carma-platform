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

package gov.dot.fhwa.saxton.carma.guidance.conflictdetector;

import cav_msgs.RouteSegment;

/**
 * A struct representing a conflict with another vehicle along a route
 * The conflict is define as a range of downtrack distances, times, a lane, and a starting route segment
 * No conflict extends across multiple lanes. Such an occurrence will be represented as multiple conflicts.
 */
public final class ConflictSpace {
  double startDowntrack;
  double endDowntrack;
  double startTime;
  double endTime;
  int lane;
  RouteSegment segment;

  /**
   * Constructor
   * 
   * @param startDowntrack The starting location on the route of this conflict
   * @param startTime The starting time of this conflict in seconds
   * @param The lane index this conflict will occur in
   * @param The route segment this conflict starts on
   */
  ConflictSpace(double startDowntrack, double startTime, int lane, RouteSegment segment) {
    this.startDowntrack = startDowntrack;
    this.startTime = startTime; 
    this.lane = lane;
    this.segment = segment;
  }

  /**
   * Set starting downtrack
   * 
   * @param startDowntrack The starting downtrack distance in meters
   */
  public void setStartDowntrack(double startDowntrack) {
    this.startDowntrack = startDowntrack;
  }

  /**
   * Set starting time
   * 
   * @param startTime The starting time in seconds
   */
  public void setStartTime(double startTime) {
    this.startTime = startTime;
  }

  /**
   * Set ending downtrack
   * 
   * @param endDowntrack The ending downtrack distance in meters
   */
  public void setEndDowntrack(double endDowntrack) {
    this.endDowntrack = endDowntrack;
  }

  /**
   * Set ending time
   * 
   * @param endTime The ending time in seconds
   */
  public void setEndTime(double endTime) {
    this.endTime = endTime;
  }

  /**
   * Set the lane
   * 
   * @param lane The lane index
   */
  public void setLane(int lane) {
    this.lane = lane;
  }

  /**
   * Set the route segment
   * 
   * @param segment The route segment this conflict starts on
   */
  public void setSegment(RouteSegment segment) {
    this.segment = segment;
  }

  /**
   * Get starting downtrack
   * 
   * @return The starting downtrack distance in meters
   */
  public double getStartDowntrack() {
    return this.startDowntrack;
  }

  /**
   * Get ending downtrack
   * 
   * @return The ending downtrack distance in meters
   */
  public double getEndDowntrack() {
    return this.endDowntrack;
  }

  /**
   * Get starting time
   * 
   * @return The starting time in seconds
   */
  public double getStartTime() {
    return this.startTime;
  }

  /**
   * Get ending time
   * 
   * @return The ending time in seconds
   */
  public double getEndTime() {
    return this.endTime;
  }

  /**
   * Get the lane
   * 
   * @return The lane index
   */
  public int getLane() {
    return this.lane;
  }

  /**
   * Set the route segment
   * 
   * @return The route segment this conflict starts on
   */
  public RouteSegment getSegment() {
    return this.segment;
  }
}