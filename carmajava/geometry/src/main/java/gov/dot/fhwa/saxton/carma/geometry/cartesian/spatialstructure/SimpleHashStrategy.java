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
 * A simple implementation of the NSpacialHashStrategy
 * 
 * The conversion is simply an long truncation of a points coordinates divided by the corresponding cell dimension
 */
public class SimpleHashStrategy implements NSpatialHashStrategy{
  private final double[] cellDims;

  /**
   * Constructor
   * 
   * @param cellDims The dimensions of a cell which this will map points to
   */
  public SimpleHashStrategy(final double[] cellDims) {
    this.cellDims = cellDims;
  }

  @Override
  public NSpatialHashKey getKey(Point point) {
    if (point.getNumDimensions() != cellDims.length) {
      return null;
    }
    final long[] key =  new long[point.getNumDimensions()];
    for (int i = 0; i < point.getNumDimensions(); i++) {
      key[i] = (long)(point.getDim(i)/cellDims[i]);
    }
    return new NSpatialHashKey(key);
  }

  @Override
  public int getNumDimensions() {
    return cellDims.length;
  }
}