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

package gov.dot.fhwa.saxton.carma.geometry.cartesian.temp;// Change

import java.util.Arrays;

/**
 * A representation of a point in N-dimensional space.
 */
public class Point {
  private double[] dimensions;

  public Point(double[] dimensions){
    dimensions = dimensions;
  }
  public Point(int size, double value){
    dimensions = new double[size];
    Arrays.fill(dimensions, value);
  }
  //Create this point as a deep copy of the previous point
  public Point(Point p1){
    dimensions = new double[p1.getNumDimensions()];
    for (int i = 0; i < p1.getNumDimensions(); i++){
      dimensions[i] = p1.getDim(i);
    }
  }

  public int getNumDimensions(){
    return dimensions.length;
  }

  public double getDim(int dimension){
    return dimensions[dimension];
  }

  public void setDim(int dimension, double value) throws IllegalArgumentException {
    if (dimension > dimensions.length - 1 || dimension < 0) {
      throw new IllegalArgumentException("Attempted to set an invalid dimension: " + dimension);
    }
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

  // We vector should have a getDim and getNumDimensions method
  public void translate(Vector vec){
    int size = (this.getNumDimensions() < vec.getNumDimensions()) ? this.getNumDimensions() : vec.getNumDimensions();
    for (int i = 0; i < size; i++){
      dimensions[i] += vec.getDim(i);
    }
  }
}
