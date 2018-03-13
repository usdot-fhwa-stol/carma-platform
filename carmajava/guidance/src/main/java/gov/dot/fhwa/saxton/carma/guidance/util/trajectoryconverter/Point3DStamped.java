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

import gov.dot.fhwa.saxton.carma.geometry.cartesian.Point3D;

public class Point3DStamped {
  Point3D point;
  double stamp;
  int segmentIdx;
  double downtrack;

  public void setPoint(Point3D point){
    this.point = point;
  }

  public void setStamp(double stamp){
    this.stamp = stamp;
  }

  public void setSegmentIdx(int segmentIdx) {
    this.segmentIdx = segmentIdx;
  }

  public void setDowntrack(double downtrack) {
    this.downtrack = downtrack;
  }

  public Point3D getPoint(){
    return point;
  }

  public double getStamp(){
    return stamp;
  }

  public int getSegmentIdx(){
    return segmentIdx;
  }

  public double getDowntrack() {
    return downtrack;
  }
}