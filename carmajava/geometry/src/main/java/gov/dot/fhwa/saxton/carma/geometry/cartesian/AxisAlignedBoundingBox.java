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
 * AxisAlignedBoundingBox provides functions for calculating if a CartesianObject intersects with
 * another CartesianObject or a point.
 * The calculations are performed assuming both objects bounds are in the same frame of reference
 * Objects with mismatched dimensions cannot be compared
 */
public class AxisAlignedBoundingBox implements IIntersectionChecker {
  @Override public boolean intersects(CartesianObject obj, Point p) throws IllegalArgumentException {
    if (obj.getNumDimensions() != p.getNumDimensions())
      throw new IllegalArgumentException("Cannot check the intersection of CartesianElements with different dimensions");
    final double[][] objBounds = obj.getBounds();
    final int minIdx = obj.getMinBoundIndx();
    final int maxIdx = obj.getMaxBoundIndx();

    for (int i = 0; i < p.getNumDimensions(); i++) {
      if (!(objBounds[i][minIdx] < p.getDim(i) && p.getDim(i) < objBounds[i][maxIdx])) {
        return false;
      }
    }

    return true;
  }

  @Override public boolean intersects(CartesianObject obj, CartesianObject obj2) throws IllegalArgumentException {
    if (obj.getNumDimensions() != obj2.getNumDimensions())
      throw new IllegalArgumentException("Cannot check the intersection of CartesianElements with different dimensions");
    final double[][] bounds1 = obj.getBounds();
    final double[][] bounds2 = obj2.getBounds();
    final int min1 = obj.getMinBoundIndx();
    final int max1 = obj.getMaxBoundIndx();
    final int min2 = obj2.getMinBoundIndx();
    final int max2 = obj2.getMaxBoundIndx();


    for (int i = 0; i < obj.getNumDimensions(); i++) {
      if (Math.max(bounds1[i][min1], bounds2[i][min2]) > Math.min(bounds1[i][max1], bounds2[i][max2])) {
        return false;
      }
    }
    return true;
  }
}
