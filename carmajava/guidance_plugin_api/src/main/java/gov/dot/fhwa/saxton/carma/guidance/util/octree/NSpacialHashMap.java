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
import gov.dot.fhwa.saxton.carma.geometry.cartesian.IIntersectionChecker;
import gov.dot.fhwa.saxton.carma.geometry.cartesian.Point;

import java.util.Arrays;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;

/**
 *  N-Dimensional Spacial Hash Map
 * 
 */
public class NSpacialHashMap implements ISpacialStructure {
  private static final int MIN_BOUND_IDX = CartesianObject.MIN_BOUND_IDX;
  private static final int MAX_BOUND_IDX = CartesianObject.MAX_BOUND_IDX;
  final private HashMap<NSpacialHashKey, List<CartesianObject>> map = new HashMap<>();
  private IIntersectionChecker intersectionChecker;
  private double[][] bounds;
  private NSpacialHashStrategy spacialHashStrategy;
  
  NSpacialHashMap(IIntersectionChecker intersectionChecker, NSpacialHashStrategy spacialHashStrategy) {
    this.intersectionChecker = intersectionChecker;
    this.spacialHashStrategy = spacialHashStrategy;
  }

  @Override
  public boolean insert(CartesianObject obj) {
    if (bounds == null) {
      bounds = Arrays.copyOf(obj.getBounds(), obj.getBounds().length);
    }
    double[][] minMaxCoordinates = obj.getMinMaxCoordinates();
    updateBounds(minMaxCoordinates);
    
    // get keys for the min and max points
    NSpacialHashKey minKey = spacialHashStrategy.getKey(new Point(minMaxCoordinates[MIN_BOUND_IDX]));
    NSpacialHashKey maxKey = spacialHashStrategy.getKey(new Point(minMaxCoordinates[MAX_BOUND_IDX]));
  
    // iterate over region
    int[] iterators = Arrays.copyOf(minKey.values, minKey.values.length);// TODO check this is deep copy
    addToCells(obj, minKey, maxKey, iterators, 0, obj.getNumDimensions());
    return true;
  }

  // Double check that this doesn't duplicate objects
  private void addToCells(CartesianObject obj, NSpacialHashKey minKey, NSpacialHashKey maxKey,
   int[] iterators, int dim, int numDims) {
    if (dim >= numDims) {
      return;
    }
    for (int i = minKey.values[dim]; i <= maxKey.values[dim]; i++) {
      addToCells(obj, minKey, maxKey, iterators, dim+1, numDims);
      iterators[i] = i;
      NSpacialHashKey key = new NSpacialHashKey(iterators);
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
      if (bounds[i][MIN_BOUND_IDX] < minMaxCoordinates[MIN_BOUND_IDX][i]) {
        bounds[i][MIN_BOUND_IDX] = minMaxCoordinates[MIN_BOUND_IDX][i];
      } else if (bounds[i][MAX_BOUND_IDX] < minMaxCoordinates[MAX_BOUND_IDX][i]) {
        bounds[i][MAX_BOUND_IDX] = minMaxCoordinates[MAX_BOUND_IDX][i];
      }
    }
  }

  @Override
  public boolean remove(CartesianObject obj) {
    double[][] minMaxCoordinates = obj.getMinMaxCoordinates();
    // get keys for the min and max points
    NSpacialHashKey minKey = spacialHashStrategy.getKey(new Point(minMaxCoordinates[MIN_BOUND_IDX]));
    NSpacialHashKey maxKey = spacialHashStrategy.getKey(new Point(minMaxCoordinates[MAX_BOUND_IDX]));
  
    // iterate over region
    int[] iterators = Arrays.copyOf(minKey.values, minKey.values.length);// TODO check this is deep copy
    removeFromCells(obj, minKey, maxKey, iterators, 0, obj.getNumDimensions());
    return true;
  }

  // Remove from cells
  private void removeFromCells(CartesianObject obj, NSpacialHashKey minKey, NSpacialHashKey maxKey,
   int[] iterators, int dim, int numDims) {
    if (dim >= numDims) {
      return;
    }
    for (int i = minKey.values[dim]; i <= maxKey.values[dim]; i++) {
      removeFromCells(obj, minKey, maxKey, iterators, dim+1, numDims);
      iterators[i] = i;
      NSpacialHashKey key = new NSpacialHashKey(iterators);
      List<CartesianObject> objects = map.get(key);
      if (objects == null) {
        continue;
      }
      objects.remove(obj);
    }
  }

  @Override
  public List<CartesianObject> getCollisions(CartesianObject obj) {
    double[][] minMaxCoordinates = obj.getMinMaxCoordinates();
    // get keys for the min and max points
    NSpacialHashKey minKey = spacialHashStrategy.getKey(new Point(minMaxCoordinates[MIN_BOUND_IDX]));
    NSpacialHashKey maxKey = spacialHashStrategy.getKey(new Point(minMaxCoordinates[MAX_BOUND_IDX]));
  
    // iterate over region
    int[] iterators = Arrays.copyOf(minKey.values, minKey.values.length);// TODO check this is deep copy
    return getCollisionsInCells(obj, minKey, maxKey, iterators, 0, obj.getNumDimensions());
  }

  private List<CartesianObject> getCollisionsInCells(CartesianObject obj, NSpacialHashKey minKey,
   NSpacialHashKey maxKey, int[] iterators, int dim, int numDims) {
    if (dim >= numDims) {
      return new LinkedList<>();
    }
    List<CartesianObject> collidingObjects = new LinkedList<>();
    for (int i = minKey.values[dim]; i <= maxKey.values[dim]; i++) {
      getCollisionsInCells(obj, minKey, maxKey, iterators, dim+1, numDims);
      iterators[i] = i;
      NSpacialHashKey key = new NSpacialHashKey(iterators);
      List<CartesianObject> objects = map.get(key);
      if (objects == null) {
        continue;
      }
      for(CartesianObject otherObject: objects) {
        if (intersectionChecker.intersects(obj, otherObject)) {
          collidingObjects.add(otherObject);
        }
      }
    }
    return collidingObjects;
  }

  @Override
  public List<CartesianObject> getCollisions(Point p) {
    // get keys for the min and max points
    NSpacialHashKey key = spacialHashStrategy.getKey(p);
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

  @Override //TODO
  public boolean encompasses(Point p) {
    if (bounds == null || bounds.length != p.getNumDimensions())
      return false; // It is impossible for a point with mismatched dimensions to definitely be enclosed
    for (int i = 0; i < bounds.length; i++) {
      if (!(bounds[i][MIN_BOUND_IDX] < p.getDim(i) && p.getDim(i) < bounds[i][MAX_BOUND_IDX])) {
        return false;
      }
    }
    return true;
  }
}

