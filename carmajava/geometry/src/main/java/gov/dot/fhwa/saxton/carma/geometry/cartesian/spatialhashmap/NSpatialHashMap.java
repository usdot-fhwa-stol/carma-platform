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

package gov.dot.fhwa.saxton.carma.geometry.cartesian.spatialhashmap;

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
 *  N-Dimensional Spatial Hash Map
 * 
 */
public class NSpatialHashMap implements ISpatialStructure {
  public static final int MIN_BOUND_IDX = CartesianObject.MIN_BOUND_IDX;
  public static final int MAX_BOUND_IDX = CartesianObject.MAX_BOUND_IDX;
  final private HashMap<NSpatialHashKey, List<CartesianObject>> map;
  private IIntersectionChecker intersectionChecker;
  private double[][] bounds;
  private NSpatialHashStrategy spatialHashStrategy;
  private final int numDimensions;
  
  public NSpatialHashMap(IIntersectionChecker intersectionChecker, NSpatialHashStrategy spatialHashStrategy) {
    this.intersectionChecker = intersectionChecker;
    this.spatialHashStrategy = spatialHashStrategy;
    this.map = new HashMap<>(); // TODO either pass this in directly or supply an initial capacity
    this.numDimensions = spatialHashStrategy.getNumDimensions();
  }

  @Override
  public boolean insert(CartesianObject obj) {
    if (obj == null || obj.getNumDimensions() != numDimensions)
      return false; // Cannot insert mismatched dimensions
    if (bounds == null) // Create bounds if this is the first object
      bounds = Arrays.copyOf(obj.getBounds(), obj.getBounds().length);
    double[][] minMaxCoordinates = obj.getMinMaxCoordinates();
    updateBounds(minMaxCoordinates);
    
    // get keys for the min and max points
    NSpatialHashKey minKey = spatialHashStrategy.getKey(new Point(minMaxCoordinates[MIN_BOUND_IDX]));
    NSpatialHashKey maxKey = spatialHashStrategy.getKey(new Point(minMaxCoordinates[MAX_BOUND_IDX]));
  
    // iterate over region
    int[] iterators = Arrays.copyOf(minKey.values, minKey.values.length);// TODO check this is deep copy
    addToCells(obj, minKey, maxKey, iterators, 0, obj.getNumDimensions());
    return true;
  }

  // Double check that this doesn't duplicate objects
  private void addToCells(CartesianObject obj, NSpatialHashKey minKey, NSpatialHashKey maxKey,
   int[] iterators, int dim, int numDims) {
    if (dim >= numDims) {
      return;
    }
    for (int i = minKey.values[dim]; i <= maxKey.values[dim]; i++) {
      iterators[dim] = i;
      addToCells(obj, minKey, maxKey, iterators, dim+1, numDims);
      NSpatialHashKey key = new NSpatialHashKey(Arrays.copyOf(iterators, iterators.length));
      List<CartesianObject> objects = map.get(key);
      if (objects == null) {
        objects = new LinkedList<>();
        map.put(key, objects);
      }
      objects.add(obj);
    }
  }

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
    int[] iterators = Arrays.copyOf(minKey.values, minKey.values.length);// TODO check this is deep copy
    removeFromCells(obj, minKey, maxKey, iterators, 0, obj.getNumDimensions());
    return true;
  }

  // Remove from cells
  private void removeFromCells(CartesianObject obj, NSpatialHashKey minKey, NSpatialHashKey maxKey,
   int[] iterators, int dim, int numDims) {
    if (dim >= numDims) {
      return;
    }
    for (int i = minKey.values[dim]; i <= maxKey.values[dim]; i++) {
      iterators[dim] = i;
      removeFromCells(obj, minKey, maxKey, iterators, dim+1, numDims);
      NSpatialHashKey key = new NSpatialHashKey(iterators);
      List<CartesianObject> objects = map.get(key);
      if (objects == null) {
        continue;
      }
      objects.remove(obj);
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
    int[] iterators = Arrays.copyOf(minKey.values, minKey.values.length);// TODO check this is deep copy
    getCollisionsInCells(obj, collidedObjects, minKey, maxKey, iterators, 0, obj.getNumDimensions());
    return new ArrayList<CartesianObject>(collidedObjects);
  }

  private void getCollisionsInCells(CartesianObject obj, HashSet<CartesianObject> collidedObjects,
  NSpatialHashKey minKey, NSpatialHashKey maxKey, int[] iterators, int dim, int numDims) {
    if (dim >= numDims) {
      return;
    }
    for (int i = minKey.values[dim]; i <= maxKey.values[dim]; i++) {
      iterators[dim] = i;
      getCollisionsInCells(obj, collidedObjects, minKey, maxKey, iterators, dim+1, numDims);
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

  protected Set<NSpatialHashKey> getKeys() {
    return map.keySet();
  }

  protected Collection<List<CartesianObject>> getObjects() {
    return map.values();
  }
}

