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

import gov.dot.fhwa.saxton.carma.geometry.cartesian.CartesianObject;
import gov.dot.fhwa.saxton.carma.geometry.cartesian.Point;
import java.util.List;

/**
 *  interface for performing inserts on an octree
 */
public interface ISpacialStructure {
  /**
   * 
   */
  boolean insert(CartesianObject obj);

    /**
   * 
   */
  boolean remove(CartesianObject obj);

      /**
   * 
   */
  List<CartesianObject> checkCollisions(CartesianObject obj);

  boolean encompasses(Point p);
}