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

import gov.dot.fhwa.saxton.carma.geometry.cartesian.CartesianObject;
import gov.dot.fhwa.saxton.carma.geometry.cartesian.Point;
import java.util.List;

/**
 * Interface for a data structure which maintains collections of objects to evaluate for collisions
 * 
 * Spacial structures may be n-dimensional but they are not cross-dimensional. 
 * You cannot evaluate a 2d point in a 3d space
 */
public interface ISpatialStructure {
  /**
   * Insert an object into the structure
   * 
   * @param obj The cartesian object which will be inserted.
   * 
   * @return true if object could be inserted. False if not
   */
  boolean insert(CartesianObject obj);

  /**
   * Remove the specified object from the structure
   * Target object is found using the Object.equals() function for that object
   * 
   * @param obj The object to remove
   * 
   * @return True if the object was removed. False if the object was not removed or was not present in the structure
   */
  boolean remove(CartesianObject obj);

  /**
   * Get a list of all CartesianObjects that this object collides with
   * 
   * @param obj The object to check collisions against
   * 
   * @return The list of colliding objects. Empty if no collision. NULL if invalid object was provided. 
   */
  List<CartesianObject> getCollisions(CartesianObject obj);

  /**
   * Get a list of all CartesianObjects that this point collides with
   * 
   * @param obj The object to check collisions against
   * 
   * @return The list of colliding objects. Empty if no collision. NULL if invalid point was provided. 
   */
  List<CartesianObject> getCollisions(Point p);

  /**
   * True if the provided point is surrounded by the spacial bounds of this structure
   * 
   * @param p The point to check
   * 
   * @return True if point is fully within bounds
   */
  boolean surrounds(Point p);

 /**
   * Gets the bounds of the space currently encompassed by this structure
   * Access max value of first dimension with bounds[0][0]
   * Access max value of first dimension with bounds[0][1]
   * 
   * @return A 2d array where the rows are the dimension and the columns are the min/max values
   */
  double[][] getBounds();

  /**
   * Returns the number of dimensions this structure encompasses
   * 
   * @return The number of dimensions
   */
  int getNumDimensions();
}