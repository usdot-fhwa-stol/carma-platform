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

package gov.dot.fhwa.saxton.carma.guidance.util.trajectoryconverter;

import gov.dot.fhwa.saxton.carma.geometry.cartesian.Point2D;
import gov.dot.fhwa.saxton.carma.geometry.cartesian.Point3D;

/**
 * Class to store points in a path returned by the TrajectoryConverter. 
 * Points have the following
 * A location on the route as (downtrack, crosstrack, time). This is stored as a point for use in ConflictDetector
 * The route segment they correspond to
 * A downtrack distance along the current segment
 */
public final class RoutePointStamped {
  Point3D point; // Downtrack, Crosstrack, and Time 
  cav_msgs.RouteSegment segment; // Segment on the route
  double segDowntrack; // Downtrack distance along current segment

  /**
   * Constructor
   * 
   * @param downtrack The downtrack distance along the route
   * @param crosstrack The cross track distance
   * @param time The time stamp
   */
  public RoutePointStamped(double downtrack, double crosstrack, double time) {
    this.point = new Point3D(downtrack, crosstrack, time);
  }

  public void setDowntrack(double downtrack) {
    this.point.setX(downtrack);
  }

  public void setCrosstrack(double crosstrack) {
    this.point.setY(crosstrack);
  }

  public void setStamp(double time) {
    this.point.setZ(time);
  }

  public void setSegment(cav_msgs.RouteSegment segment) {
    this.segment = segment;
  }

  public void setSegDowntrack(double segDowntrack) {
    this.segDowntrack = segDowntrack;
  }

  public Point3D getPoint(){
    return point;
  }

  public double getDowntrack() {
    return this.point.getX();
  }

  public double getCrosstrack() {
    return this.point.getY();
  }

  public double getStamp() {
    return this.point.getZ();
  }

  public double getSegDowntrack() {
    return this.segDowntrack;
  }

  public cav_msgs.RouteSegment getSegment(){
    return segment;
  }
}