/*
 * Copyright (C) 2018-2020 LEIDOS.
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

package gov.dot.fhwa.saxton.carma.geometry.cartesian;

/**
 * Interface defining the methods which are needed to check for intersections between cartesian objects
 */
public interface IIntersectionChecker {
  /**
   * True if the provided point intersects (is inside) the provided cartesian object
   * @param obj The cartesian object to be checked
   * @param p The point to be checked. Must be of the same dimension as the cartesian object
   * @return True if obj and p intersect
   * @throws IllegalArgumentException Thrown if obj and p are not of the same dimension
   */
  boolean intersects(CartesianObject obj, Point p) throws IllegalArgumentException ;

  /**
   * True if the first cartesian object intersects the second cartesian object
   * @param obj The first cartesian object to be checked
   * @param obj2 The second cartesian object to be checked. Must be of the same dimension as other object
   * @return True if obj and p intersect
   * @throws IllegalArgumentException Thrown if obj and obj2 are not of the same dimension
   */
  boolean intersects(CartesianObject obj, CartesianObject obj2) throws IllegalArgumentException ;
}
