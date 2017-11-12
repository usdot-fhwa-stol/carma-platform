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

import gov.dot.fhwa.saxton.carma.geometry.cartesian.Point3D;

import java.util.List;

/**
 * Geometric representation of a lane
 */
public class Lane {
  List<LaneSegment> segments;

  /**
   * TODO Implement
   * True if the specified point can be considered in the lane
   * @param point point to compare
   * @return true if in lane
   */
  boolean inLane(Point3D point) {
    return false;
  }

  /**
   * TODO Implement
   * True if the specified obstacle can be considered in the lane
   * @param obstacle obstacle to compare
   * @return true if in lane
   */
  boolean inLane(IObstacle obstacle) {
    return false;
  }

  /**
   * Returns the length of a lane
   * @return length in meters
   */
  double length() {
    double sum = 0;
    for (LaneSegment seg : segments) {
      sum += seg.length();
    }

    return sum;
  }
}
