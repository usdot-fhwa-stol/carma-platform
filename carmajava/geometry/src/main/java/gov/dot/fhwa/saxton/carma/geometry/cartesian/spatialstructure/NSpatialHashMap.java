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
import gov.dot.fhwa.saxton.carma.geometry.cartesian.IIntersectionChecker;
import gov.dot.fhwa.saxton.carma.geometry.cartesian.Point;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Set;

/**
 * N-Dimensional Spatial Hash Map
 * Allows for fast collision lookup and object insertion
 * 
 * Collision checking is evaluated with the provided IIntersectionChecker
 * Objects are hashed using the provided NSpatialHashStrategy
 * This class is not thread safe on its own
 */
public class NSpatialHashMap implements ISpatialStructure {
  public static final int MIN_BOUND_IDX = CartesianObject.MIN_BOUND_IDX;
  public static final int MAX_BOUND_IDX = CartesianObject.MAX_BOUND_IDX;
  final private Map<NSpatialHashKey, List<CartesianObject>> map;
  private IIntersectionChecker intersectionChecker;
  private double[][] bounds;
  private NSpatialHashStrategy spatialHashStrategy;
  private final int numDimensions;
  
  /**
   * Constructor
   * 
   * @param intersectionChecker The intersection testing method that will be used
   * @param NSpatialHashStrategy The method used to generate a Key (spatial cell) for a given object
   */
  public NSpatialHashMap(IIntersectionChecker intersectionChecker,
   NSpatialHashStrategy spatialHashStrategy, Map<NSpatialHashKey, List<CartesianObject>> map) {
    this.intersectionChecker = intersectionChecker;
    this.spatialHashStrategy = spatialHashStrategy;
    this.map = map;
    this.numDimensions = spatialHashStrategy.getNumDimensions();
  }

  @Override
  public boolean insert(CartesianObject obj) {
    if (obj == null || obj.getNumDimensions() != numDimensions)
      return false; // Cannot insert mismatched dimensions
    if (bounds == null) { // Create bounds if this is the first object
      bounds = Arrays.copyOf(obj.getBounds(), obj.getBounds().length);
    }
    
    double[][] minMaxCoordinates = obj.getMinMaxCoordinates();
    updateBounds(minMaxCoordinates);
    
    // get keys for the min and max points
    NSpatialHashKey minKey = spatialHashStrategy.getKey(new Point(minMaxCoordinates[MIN_BOUND_IDX]));
    NSpatialHashKey maxKey = spatialHashStrategy.getKey(new Point(minMaxCoordinates[MAX_BOUND_IDX]));
    
    // iterate over all cells
    addToCells(obj, minKey, maxKey, 0, Arrays.copyOf(minKey.values, minKey.values.length));
    return true;
  }

  /**
   * Helper function to add objects to there respective cells
   * This function recuses in the form of n-nested for loops to add an object to all cells it overlaps
   * 
   * @param obj The object to be added
   * @param minKey The minimum key (cell) this object will be added to
   * @param maxKey The maximum key (cell) this object will be added to
   * @param dim The current dimension being processed. Can also be thought of as loop depth
   */
  private void addToCells(CartesianObject obj, NSpatialHashKey minKey, NSpatialHashKey maxKey, int dim, long[] iterators) {
	if (dim >= numDimensions) {
      return;
    }
    for (long i = minKey.values[dim]; i <= maxKey.values[dim]; i++) {
      iterators[dim] = i;
      addToCells(obj, minKey, maxKey, dim+1, iterators);
      // Only add keys when at the base of recursion
      if (dim == numDimensions - 1) {
        NSpatialHashKey key = new NSpatialHashKey(Arrays.copyOf(iterators, iterators.length));
        List<CartesianObject> objects = map.get(key);
        if (objects == null) {
          objects = new LinkedList<>();
          map.put(key, objects);
        }
        objects.add(obj);
      }
    }
  }

  /**
   * Helper function updates the hash map bounds when a new object is inserted
   * 
   * @param minMaxCoordinates The minimum and maximum coordinates of the new object. Array is a transpose of that objects bounds
   */
  private void updateBounds(double[][] minMaxCoordinates) {
    for (int i = 0; i < minMaxCoordinates[0].length; i++) {
      if (minMaxCoordinates[MIN_BOUND_IDX][i] < bounds[i][MIN_BOUND_IDX]) {
        bounds[i][MIN_BOUND_IDX] = minMaxCoordinates[MIN_BOUND_IDX][i];
      } 
      if (minMaxCoordinates[MAX_BOUND_IDX][i] > bounds[i][MAX_BOUND_IDX]) {
        bounds[i][MAX_BOUND_IDX] = minMaxCoordinates[MAX_BOUND_IDX][i];
      }
    }
  }

  @Override
  public boolean remove(CartesianObject obj) {
    if (bounds == null || obj == null || obj.getNumDimensions() != numDimensions)
      return false; // Return false if nothing added yet or attempting to remove object of mismatched dimensions.

    double[][] minMaxCoordinates = obj.getMinMaxCoordinates();
    // get keys for the min and max points
    NSpatialHashKey minKey = spatialHashStrategy.getKey(new Point(minMaxCoordinates[MIN_BOUND_IDX]));
    NSpatialHashKey maxKey = spatialHashStrategy.getKey(new Point(minMaxCoordinates[MAX_BOUND_IDX]));
  
    // iterate over region
    removeFromCells(obj, minKey, maxKey, 0, Arrays.copyOf(minKey.values, minKey.values.length));
    return true;
  }

  /**
   * Helper function to remove objects to there respective cells
   * This function recuses in the form of n-nested for loops to remove an object from all cells it overlaps
   * 
   * @param obj The object to be added
   * @param minKey The minimum key (cell) this object will be added to
   * @param maxKey The maximum key (cell) this object will be added to
   * @param dim The current dimension being processed. Can also be thought of as loop depth
   */
  private void removeFromCells(CartesianObject obj, NSpatialHashKey minKey, NSpatialHashKey maxKey, int dim, long[] iterators) {
    if (dim >= numDimensions) {
      return;
    }
    for (long i = minKey.values[dim]; i <= maxKey.values[dim]; i++) {
      iterators[dim] = i;
      removeFromCells(obj, minKey, maxKey, dim+1, iterators);
      if (dim == numDimensions - 1) {
        NSpatialHashKey key = new NSpatialHashKey(iterators);
        List<CartesianObject> objects = map.get(key);
        if (objects == null) {
          continue;
        }
        objects.remove(obj);
      }
    }
  }

  @Override
  public List<CartesianObject> getCollisions(CartesianObject obj) {
    if (obj == null || obj.getNumDimensions() != numDimensions)
      return null; // Return null if nothing added yet or attempting to remove object of mismatched dimensions.
    if (bounds == null)
      return new LinkedList<>(); // Returned no conflicts as no other objects are present
    
    double[][] minMaxCoordinates = obj.getMinMaxCoordinates();
    // get keys for the min and max points
    NSpatialHashKey minKey = spatialHashStrategy.getKey(new Point(minMaxCoordinates[MIN_BOUND_IDX]));
    NSpatialHashKey maxKey = spatialHashStrategy.getKey(new Point(minMaxCoordinates[MAX_BOUND_IDX]));
  
    // iterate over region
    HashSet<CartesianObject> collidedObjects = new HashSet<>();
    getCollisionsInCells(obj, collidedObjects, minKey, maxKey, 0, Arrays.copyOf(minKey.values, minKey.values.length));
    return new ArrayList<CartesianObject>(collidedObjects);
  }

  /**
   * Helper function to determine all objects the provided object collides with
   * This function recuses in the form of n-nested for loops to identify all colliding objects
   * 
   * @param obj The object to be added
   * @param collidedObjects A hash set of all previously detected collisions
   * @param minKey The minimum key (cell) this object will be added to
   * @param maxKey The maximum key (cell) this object will be added to
   * @param iterators An array containing the current value of each nest for loop iterator
   * @param dim The current dimension being processed. Can also be thought of as loop depth
   */
  private void getCollisionsInCells(CartesianObject obj, HashSet<CartesianObject> collidedObjects,
  NSpatialHashKey minKey, NSpatialHashKey maxKey, int dim, long[] iterators) {
    if (dim >= numDimensions) {
      return;
    }
    for (long i = minKey.values[dim]; i <= maxKey.values[dim]; i++) {
      iterators[dim] = i;
      getCollisionsInCells(obj, collidedObjects, minKey, maxKey, dim+1, iterators);
      // Only add objects at bottom of recursion
      if (dim == numDimensions - 1) {
        NSpatialHashKey key = new NSpatialHashKey(iterators);
        List<CartesianObject> objects = map.get(key);
        if (objects == null) {
          continue;
        }
        for(CartesianObject otherObject: objects) {
          if (intersectionChecker.intersects(obj, otherObject)) {
            collidedObjects.add(otherObject);
          }
        }
      }
    }
  }

  @Override
  public List<CartesianObject> getCollisions(Point p) {
    if (p == null || p.getNumDimensions() != numDimensions)
      return null; // Return null if nothing added yet or attempting to remove object of mismatched dimensions.
    if (bounds == null)
      return new LinkedList<>(); // Returned no conflicts as no other objects are present
    
    // get keys for the min and max points
    NSpatialHashKey key = spatialHashStrategy.getKey(p);
    List<CartesianObject> collidingObjects = new LinkedList<>();
    List<CartesianObject> objects = map.get(key);
      if (objects == null) {
        return new LinkedList<>();
      }
      for(CartesianObject otherObject: objects) {
        if (intersectionChecker.intersects(otherObject, p)) {
          collidingObjects.add(otherObject);
        }
      }
    return collidingObjects;
  }

  @Override 
  public boolean surrounds(Point p) {
    if (p == null || bounds == null || p.getNumDimensions() != numDimensions)
      return false; // It is impossible for a point with mismatched dimensions to definitely be enclosed
    for (int i = 0; i < bounds.length; i++) {
      if (!(bounds[i][MIN_BOUND_IDX] < p.getDim(i) && p.getDim(i) < bounds[i][MAX_BOUND_IDX])) {
        return false;
      }
    }
    return true;
  }

  @Override
  public int getNumDimensions() {
    return numDimensions;
  }

  @Override
  public double[][] getBounds() {
    return bounds;
  }

  /**
   * Helper function for unit testing
   * Extreme care should be taken if used outside a testing framework
   * 
   * @return A set of the keys currently in this hashmap
   */
  protected Set<NSpatialHashKey> getKeys() {
    return map.keySet();
  }

  /**
   * Helper function for unit testing
   * Extreme care should be taken if used outside a testing framework
   * 
   * @return The collection of objects in each cell
   */
  protected Collection<List<CartesianObject>> getObjects() {
    return map.values();
  }
}

