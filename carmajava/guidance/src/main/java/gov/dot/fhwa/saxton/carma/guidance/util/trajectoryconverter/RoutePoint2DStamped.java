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
 * 2D location in a segment frame
 * A time stamp in seconds
 * The route segment they correspond to
 * A total downtrack distance along a route
 */
public final class RoutePoint2DStamped {
  Point2D point; // Segment Downtrack and Crosstrack position in meters
  double stamp; // Time in seconds
  cav_msgs.RouteSegment segment; // Segment on the route
  double downtrack; // Total downtrack distance in m

  public void setPoint(Point2D point){
    this.point = point;
  }

  public void setStamp(double stamp){
    this.stamp = stamp;
  }

  public void setSegment(cav_msgs.RouteSegment segment) {
    this.segment = segment;
  }

  public void setDowntrack(double downtrack) {
    this.downtrack = downtrack;
  }

  public Point2D getPoint(){
    return point;
  }

  public double getStamp(){
    return stamp;
  }

  public cav_msgs.RouteSegment getSegment(){
    return segment;
  }

  public double getDowntrack() {
    return downtrack;
  }
}