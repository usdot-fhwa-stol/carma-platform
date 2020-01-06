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
 * A representation of a point in N-dimensional space.
 */
public class Point implements CartesianElement {
  protected double[] dimensions;

  /**
   * Constructor defines this point with values from the provided array
   * Each value corresponds to the specified dimension with 1 indexing.
   * Such that index 0 in the array specified the value of dimension 1 (x axis)
   * @param dimensions An array of dimension values
   */
  public Point(double... dimensions){
    this.dimensions = dimensions;
  }

  /**
   * Constructor defines this point as a deep copy of the provided point
   * @param p1 The point to be copied
   */
  public Point(Point p1){
    this.dimensions = new double[p1.getNumDimensions()];
    for (int i = 0; i < this.dimensions.length; i++){
      this.dimensions[i] = p1.getDim(i);
    }
  }

  @Override public int getNumDimensions(){
    return dimensions.length;
  }

  /**
   * Returns the value corresponding to the specified dimension
   * @param dimension The dimension to return
   * @return The value of the specified dimension
   */
  public double getDim(int dimension){
    return dimensions[dimension];
  }

  /**
   * Sets the value of the specified dimension
   * @param dimension The dimension to set
   * @param value The value to apply
   */
  public void setDim(int dimension, double value) {
    dimensions[dimension] = value;
  }

  /**
   * Calculates the euclidean distance from this point to the provided point
   * @param p2 The point to calculate distance from
   * @return The calculated euclidean distance
   */
  public double distanceFrom(Point p2) throws IllegalArgumentException {
    if (this.getNumDimensions() != p2.getNumDimensions()) {
      throw new IllegalArgumentException("Point dimensions do not match");
    }
    int size = this.getNumDimensions();
    double squareSum = 0;
    for (int i = 0; i < size; i++){
      double diff = this.getDim(i) - p2.getDim(i);
      squareSum += diff*diff;
    }
    return Math.sqrt(squareSum);
  }

  /**
   * Returns true if two points are equivalent within the specified delta
   * @param p The point to compare against this point
   * @param delta The delta to use as the margin of error
   * @return True if points are equal within delta
   */
  public boolean almostEquals(Point p, double delta) {
    int numDim = this.getNumDimensions() < p.getNumDimensions() ? this.getNumDimensions() : p.getNumDimensions();

    for (int i = 0; i < numDim; i++) {
      if (Math.abs(this.getDim(i) - p.getDim(i)) > delta) {
        return false;
      }
    }

    return true;
  }

  @Override public String toString() {
    String str = this.getClass().getSimpleName() + " (";
    for (int i = 0; i < dimensions.length; i++) {
      str = str.concat(" " + dimensions[i]);
      if (i != dimensions.length - 1) {
        str = str.concat(",");
      }
    }
    return str.concat(" )");
  }
}
