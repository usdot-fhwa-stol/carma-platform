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

package gov.dot.fhwa.saxton.carma.guidance.util.octree;

import gov.dot.fhwa.saxton.carma.geometry.cartesian.Point3D;

/**
 * Basic data class for the {@link IntervalTree}
 * <p>
 * Stores a generic data field the start and end points of the corresponding interval.
 * The inclusive/exclusiveness of those start and end points is determined by the IntervalTree's
 * {@link IntervalCalculatorStrategy}
 */
public class OcTreeDatum<T> {
  private Point3D point;
  private T data;

  /**
   * Construct an interval containing a data element
   * <p>
   * Start must not be greater than end, will throw an ArithmeticException if attempted
   */
  public OcTreeDatum(Point3D point, T data) {
    this.data = data;
    this.point = point;
  }

  /**
   * Get the data value of this datum
   */
  public T getData() {
    return data;
  }

  public Point3D getPoint() {
    return this.point;
  }

  @Override
  public String toString() {
    return "OcTreeDatum{data:" + data + ", point: " + point + "}";
  }
}
