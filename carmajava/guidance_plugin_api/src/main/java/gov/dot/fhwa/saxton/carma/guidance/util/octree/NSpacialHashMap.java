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

import java.util.Arrays;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;

/**
 *  interface for performing inserts on an octree
 */
public class NSpacialHashMap implements ISpacialStructure {
  private HashMap<HashKey, List<CartesianObject>> map;
  private double[] cellDims;
  
  NSpacialHashMap(double[] cellDims) {
    this.cellDims = cellDims;
  }

  @Override
  public boolean insert(CartesianObject obj) {
    final int numDims = obj.getNumDimensions();
    double[][] bounds = obj.getBounds();
    double[] minPoint = new double[bounds.length];
    double[] maxPoint = new double[bounds.length];

    for (int i = 0; i < numDims; i++) {
      minPoint[i] = bounds[i][CartesianObject.MIN_BOUND_IDX];
      maxPoint[i] = bounds[i][CartesianObject.MAX_BOUND_IDX];
    }
    // get keys for the min and max points

    HashKey minKey = getKey(new Point(minPoint));
    HashKey maxKey = getKey(new Point(maxPoint));
  
    // iterate over region
    int[] iterators = Arrays.copyOf(minKey.values, minKey.values.length);// TODO check this is deep copy
    addToCells(obj, minKey, maxKey, iterators, 0, numDims);
    return true;
  }

  // Double check that this doesn't duplicate objects
  private void addToCells(CartesianObject obj, HashKey minKey, HashKey maxKey, int[] iterators, int dim, int numDims) {
    if (dim >= numDims) {
      return;
    }
    for (int i = minKey.values[dim]; i <= maxKey.values[dim]; i++) {
      addToCells(obj, minKey, maxKey, iterators, dim+1, numDims);
      iterators[i] = i;
      HashKey key = new HashKey(iterators);
      List<CartesianObject> objects = map.get(key);
      if (objects == null) {
        objects = new LinkedList<>();
        map.put(key, objects);
      }
      objects.add(obj);
    }
  }

  @Override
  boolean remove(CartesianObject obj) {

  }

  @Override
  List<CartesianObject> checkCollisions(CartesianObject obj) {

  }

  private HashKey getKey(Point point) {
    final int[] key =  new int[point.getNumDimensions()];
    for (int i = 0; i < point.getNumDimensions(); i++) {
      key[i] = (int)(point.getDim(i)/cellDims[i]);
    }
    return new HashKey(key);
  }

  private final class HashKey {
    final int[] values;
    HashKey(int[] values) {
      this.values = values;
    }
    @Override
    public int	hashCode() {
      return Arrays.hashCode(values);
    }
  }
}

