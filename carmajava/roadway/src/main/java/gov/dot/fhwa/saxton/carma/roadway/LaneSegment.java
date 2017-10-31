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

package gov.dot.fhwa.saxton.carma.roadway;

import cav_msgs.LaneEdgeType;
import gov.dot.fhwa.saxton.carma.geometry.cartesian.Point;
import gov.dot.fhwa.saxton.carma.geometry.cartesian.Point3D;

/**
 * Represents a section of lane as a straight line with a width and lane markings
 */
public class LaneSegment {
  Point3D uptrackPoint;
  Point3D downtrackPoint;
  LaneEdgeType leftSideType;
  LaneEdgeType rightSideType;
  double width;

  public LaneSegment(Point3D uptrackPoint, Point3D downtrackPoint, LaneEdgeType leftSideType,
    LaneEdgeType rightSideType, double width) {
    this.uptrackPoint = uptrackPoint;
    this.downtrackPoint = downtrackPoint;
    this.leftSideType = leftSideType;
    this.rightSideType = rightSideType;
    this.width = width;
  }

  double length() {
    return uptrackPoint.distanceFrom(downtrackPoint);
  }

  public Point3D getUptrackPoint() {
    return uptrackPoint;
  }

  public void setUptrackPoint(Point3D uptrackPoint) {
    this.uptrackPoint = uptrackPoint;
  }

  public Point3D getDowntrackPoint() {
    return downtrackPoint;
  }

  public void setDowntrackPoint(Point3D downtrackPoint) {
    this.downtrackPoint = downtrackPoint;
  }

  public LaneEdgeType getLeftSideType() {
    return leftSideType;
  }

  public void setLeftSideType(LaneEdgeType leftSideType) {
    this.leftSideType = leftSideType;
  }

  public LaneEdgeType getRightSideType() {
    return rightSideType;
  }

  public void setRightSideType(LaneEdgeType rightSideType) {
    this.rightSideType = rightSideType;
  }

  public double getWidth() {
    return width;
  }

  public void setWidth(double width) {
    this.width = width;
  }
}
