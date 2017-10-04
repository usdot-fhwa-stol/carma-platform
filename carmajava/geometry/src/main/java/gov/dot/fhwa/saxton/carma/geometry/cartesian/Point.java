/*
 * Copyright (C) 2017 Michael McConnell.
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

package gov.dot.fhwa.saxton.carma.geometry.cartesian;// Change

import java.util.Arrays;

/**
 * A representation of a point in N-dimensional space.
 */
public class Point implements CartesianElement {
  protected double[] dimensions;

  public Point(double... dimensions){
    this.dimensions = dimensions;
  }

  public Point(int size, double value){
    this.dimensions = new double[size];
    Arrays.fill(this.dimensions, value);
  }
  //Create this point as a deep copy of the previous point
  public Point(Point p1){
    this.dimensions = new double[p1.getNumDimensions()];
    for (int i = 0; i < p1.getNumDimensions(); i++){
      this.dimensions[i] = p1.getDim(i);
    }
  }

  public int getNumDimensions(){
    return dimensions.length;
  }

  public double getDim(int dimension){
    return dimensions[dimension];
  }

  public void setDim(int dimension, double value) {
    dimensions[dimension] = value;
  }

  public double distanceFrom(Point p2){
    int size = (this.getNumDimensions() < p2.getNumDimensions()) ? this.getNumDimensions() : p2.getNumDimensions();
    double squareSum = 0;
    for (int i = 0; i < size; i++){
      squareSum += Math.pow(this.getDim(i) - p2.getDim(i), 2);
    }
    return Math.sqrt(squareSum);
  }

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
    String str = super.toString() + " (";
    for (int i = 0; i < dimensions.length; i++) {
      str.concat(" " + i + ",");
    }
    str.concat(" ): (");
    for (int i = 0; i < dimensions.length; i++) {
      str.concat(" " + dimensions[i] + ",");
    }
    return str.concat(" )");
  }
}
