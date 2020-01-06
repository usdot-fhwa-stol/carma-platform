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

package gov.dot.fhwa.saxton.carma.geometry.cartesian.spatialstructure;

import gov.dot.fhwa.saxton.carma.geometry.cartesian.Point;

/**
 * Interface for a hashing strategy used to generate cell coordinates for an NSpacialHashMap
 */
public interface NSpatialHashStrategy {
  /**
   * Gets the corresponding key for the provided point.
   * 
   * @param point The point to convert
   * 
   * @return The new key. Null if point.getNumDimensions != this.getNumDimensions
   */
  NSpatialHashKey getKey(Point point);

  /**
   * Returns the number of dimensions this strategy is equipped to handle
   * 
   * @return The number of dimensions
   */
  int getNumDimensions();
}