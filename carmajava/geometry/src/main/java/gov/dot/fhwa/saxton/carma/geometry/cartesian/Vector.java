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

package gov.dot.fhwa.saxton.carma.geometry.cartesian;// Change

/**
 * A representation of a vector in N-dimensional space.
 * While a vector can be calculated form a head and tail point,
 * it is always represented only as the distances between these two points.
 */
public class Vector implements CartesianElement {
  protected Point headPoint_;

  /**
   * Constructor defines a vector from the tail point to the head point
   * @param head The head of the vector (the arrow when drawing)
   * @param tail The tail of the vector. Must be the same dimension as the head
   * @throws IllegalArgumentException Thrown if the dimensions of the two points do not match
   */
  public Vector(Point tail, Point head) throws IllegalArgumentException {
    if (head.getNumDimensions() != tail.getNumDimensions()) {
      throw new IllegalArgumentException("Point dimensions do not match");
    }

    headPoint_ = new Point(new double[head.getNumDimensions()]);
    int size = head.getNumDimensions();
    for(int i = 0; i < size; i++){
      headPoint_.setDim(i, head.getDim(i) - tail.getDim(i));
    }
  }

  /**
   * Constructor defines a vector from an origin to the provided head point.
   * The origin is in the frame used to define the provided head point
   * @param head The head point of the vector. Point is deep copied.
   */
  public Vector(Point head){
    headPoint_ = new Point(head);
  }

  /**
   * Defines a vector as a deep copy of the provided vector.
   * @param vec The vector to copy
   */
  public Vector(Vector vec){
    // Create this vector as a deep copy of input vector
    headPoint_ = new Point(vec.headPoint_);
  }

  /**
   * Returns the magnitude of the vector
   * @return Magnitude of vector
   */
  public double magnitude(){
    return headPoint_.distanceFrom(new Point(new double[getNumDimensions()]));
  }

  /**
   * Adds two vectors.
   * If vectors are not of the same dimensions the resulting vector will have the size of this vector.
   * @param v2 The vector to add
   * @return Vector resulting from addition of the two vectors
   */
  public Vector add(Vector v2) throws IllegalArgumentException {
    if (this.getNumDimensions() != v2.getNumDimensions()) {
      throw new IllegalArgumentException("Vector dimensions do not match");
    }
    Vector newVec = new Vector(this);
    int size = this.getNumDimensions();
    for (int i = 0; i < size; i++){
      newVec.setDim(i, v2.getDim(i) + newVec.getDim(i));
    }
    return newVec;
  }

  /**
   * Subtracts provided vector from this vector.
   * If vectors are not of the same dimensions the resulting vector will have the size of this vector.
   * @param v2 The vector to subtract
   * @return Vector resulting from subtraction of the two vectors
   */
  public Vector subtract(Vector v2) throws IllegalArgumentException {
    if (this.getNumDimensions() != v2.getNumDimensions()) {
      throw new IllegalArgumentException("Vector dimensions do not match");
    }
    int size = this.getNumDimensions();
    Vector newVec = new Vector(this);
    for (int i = 0; i < size; i++){
      newVec.setDim(i, newVec.getDim(i) - v2.getDim(i));
    }
    return newVec;
  }

  /**
   * Performs scalar multiplication on this vector
   * @param s The scalar to multiply by
   * @return The resulting vector
   */
  public Vector scalarMultiply(double s){
    Vector newVec = new Vector(this);
    for (int i = 0; i < this.getNumDimensions(); i++){
      newVec.setDim(i, s * newVec.getDim(i));
    }
    return newVec;
  }

  /**
   * Performs an element wide multiplication of this vector with the provided vector.
   * If vectors are not of the same dimensions the resulting vector will have the size of this vector.
   * @param v2 The vector to multiply
   * @return Vector resulting from element wide multiplication of the two vectors
   */
  public Vector elementWiseMultiply(Vector v2) {
    int size = (this.getNumDimensions() < v2.getNumDimensions()) ? this.getNumDimensions() : v2.getNumDimensions();
    Vector newVec = new Vector(this);
    for (int i = 0; i < size; i++){
      newVec.setDim(i, v2.getDim(i) * newVec.getDim(i));
    }
    return newVec;
  }

  /**
   * Computes dot product of two vectors
   * @param v2 The vector to dot with this vector. Must be same dimension as this vector
   * @return The result fo the dot product.
   * @throws IllegalArgumentException Thrown if the dimensions of this vector and the provided vector do not match.
   */
  public double dot(Vector v2) throws IllegalArgumentException {
    if (this.getNumDimensions() != v2.getNumDimensions()) {
      throw new IllegalArgumentException("Vector dimensions do not match");
    }
    double sum = 0;

    for (int i=0; i < this.getNumDimensions(); i++){
      sum += this.getDim(i) * v2.getDim(i);
    }

    return sum;
  }

  /**
   * Calculates the angle between two vectors.
   * @param vec2 the second vector
   * @return The angle in rad between the two vectors
   */
  public double getAngleBetweenVectors(Vector vec2) {
    double vec1Mag = this.magnitude();
    double vec2Mag = vec2.magnitude();
    if (vec1Mag == 0 || vec2Mag == 0) {
      return 0;
    }
    return  Math.acos(this.dot(vec2) / (this.magnitude() * vec2.magnitude()));
  }

  /**
   * Converts this vector into a point where the frame of the point has an origin at the vector tail.
   * @return the point
   */
  public Point toPoint(){
    return new Point(headPoint_);
  }

  /**
   * Sets the value of a dimension of this vector
   * @param dimension The dimension to set
   * @param value The value to apply
   */
  public void setDim(int dimension, double value) {
    headPoint_.setDim(dimension, value);
  }

  /**
   * Gets the unit vector of this vector
   * @return The unit vector
   */
  public Vector getUnitVector(){
    return this.scalarMultiply(1.0 / this.magnitude());
  }

  @Override public int getNumDimensions(){
    return headPoint_.getNumDimensions();
  }

  /**
   * Gets the value of the requested dimension
   * @param dim Dimension to retrieve
   * @return The value of the requested dimension
   */
  public double getDim(int dim){
    return headPoint_.getDim(dim);
  }

  /**
   * Returns a new vector composed of the minimums of the two vectors
   */
  public static Vector min(Vector vec1, Vector vec2) throws IllegalArgumentException {
    if (vec1.getNumDimensions() != vec2.getNumDimensions()) {
      throw new IllegalArgumentException("Cannot compare vectors of unequal dimensions");
    }

    double[] mins = new double[vec1.getNumDimensions()];
    for (int i = 0; i < mins.length; i++) {
      mins[i] = Math.min(vec1.getDim(i), vec2.getDim(i));
    }
    return new Vector(new Point(mins));
  }

  /**
   * Returns a new vector composed of the minimums of the two vectors
   */
  public static Vector max(Vector vec1, Vector vec2) throws IllegalArgumentException {
    if (vec1.getNumDimensions() != vec2.getNumDimensions()) {
      throw new IllegalArgumentException("Cannot compare vectors of unequal dimensions");
    }

    double[] max = new double[vec1.getNumDimensions()];
    for (int i = 0; i < max.length; i++) {
      max[i] = Math.max(vec1.getDim(i), vec2.getDim(i));
    }
    return new Vector(new Point(max));
  }
}
