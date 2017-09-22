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

/**
 * A representation of a point in N-dimensional space.
 */
public class Vector {
  private Point headPoint_;

  public Vector(Point head, Point tail) throws IllegalArgumentException {
    if (head.getNumDimensions() != tail.getNumDimensions()) {
      throw new IllegalArgumentException("Point dimensions do not match");
    }

    for(int i = 0; i < head.getNumDimensions(); i++){
      headPoint_.setDim(i, head.getDim(i) - tail.getDim(i));
    }
  }

  // Assumes the vector tail is at the origin
  public Vector(Point head){
    //Does this need to be a deep copy?
    headPoint_ = head;
  }

  public Vector(Vector vec){
    // Create this vector as a deep copy of input vector
    headPoint_ = new Point(vec.getNumDimensions(), 0);
    for(int i = 0; i < headPoint_.getNumDimensions(); i++){
      headPoint_.setDim(i, vec.getDim(i));
    }
  }

  public double magnitude(){
    return headPoint_.distanceFrom(new Point(this.getNumDimensions(), 0));
  }


  public Vector add(Vector v2) throws IllegalArgumentException {
    if (this.getNumDimensions() != v2.getNumDimensions()) {
      throw new IllegalArgumentException("Vector dimensions do not match");
    }
    Vector newVec = new Vector(this);
    for (int i = 0; i < this.getNumDimensions(); i++){
      newVec.setDim(i, v2.getDim(i) + newVec.getDim(i));
    }
    return newVec;
  }

  public Vector add(double val){
    Vector newVec = new Vector(this);
    for (int i = 0; i < this.getNumDimensions(); i++){
      newVec.setDim(i, val + newVec.getDim(i));
    }
    return newVec;
  }

  public Vector scalarMultiply(double s){
    Vector newVec = new Vector(this);
    for (int i = 0; i < this.getNumDimensions(); i++){
      newVec.setDim(i, s * newVec.getDim(i));
    }
    return newVec;
  }

  public Vector scalarMultiply(Vector v2) throws IllegalArgumentException {
    if (this.getNumDimensions() != v2.getNumDimensions()) {
      throw new IllegalArgumentException("Vector dimensions do not match");
    }
    Vector newVec = new Vector(this);
    for (int i = 0; i < this.getNumDimensions(); i++){
      newVec.setDim(i, v2.getDim(i) * newVec.getDim(i));
    }
    return newVec;
  }

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

  public Point toPoint(){
    return new Point(headPoint_);
  }

  // Should this not be included given that is it specific to 3D vectors?
  // Should there be a Vec3D instead?
  public Vector cross(Vector v2) throws IllegalArgumentException {
    if (this.getNumDimensions() != 3) {
      throw new IllegalArgumentException("Cross product can only be calculated for 3d space");
    }

    if (this.getNumDimensions() != v2.getNumDimensions()) {
      throw new IllegalArgumentException("Vector dimensions do not match");
    }

    double x = this.getDim(1) * v2.getDim(2) - this.getDim(2) * v2.getDim(1);
    double y = this.getDim(2) * v2.getDim(0) - this.getDim(0) * v2.getDim(2);
    double z = this.getDim(0) * v2.getDim(1) - this.getDim(1) * v2.getDim(0);

    return new Vector(new Point3D(x,y,z));
  }

  public void setDim(int dimension, double value) throws IllegalArgumentException {
    headPoint_.setDim(dimension, value);
  }

  public Vector getUnitVector(){
    return this.scalarMultiply(1.0 / this.magnitude());
  }

  public int getNumDimensions(){
    return headPoint_.getNumDimensions();
  }

  public double getDim(int dim){
    return headPoint_.getDim(dim);
  }
}
