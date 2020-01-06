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
 * A representation of a point in 3-dimensional space.
 */
public class Point3D extends Point{

  private static final int X_DIM = 0;
  private static final int Y_DIM = 1;
  private static final int Z_DIM = 2;

  /**
   * Constructor defines a 3d point with the provided x and y values
   * @param x x-axis value
   * @param y y-axis value
   * @param z z-axis value
   */
  public Point3D(double x, double y, double z){
    super(0, 0, 0); // Ensure there is space for x,y,z
    this.dimensions[X_DIM] = x;
    this.dimensions[Y_DIM] = y;
    this.dimensions[Z_DIM] = z;
  }

  /**
   * Constructor defines this point as a deep copy of the provided point
   * @param p The point to copy
   */
  public Point3D(Point3D p) {
    super(p);
  }

  /**
   * Gets the x-axis value
   * @return x-axis value
   */
  public double getX(){
    return this.dimensions[X_DIM];
  }

  /**
   * Gets the y-axis value
   * @return y-axis value
   */
  public double getY(){
    return this.dimensions[Y_DIM];
  }

  /**
   * Gets the z-axis value
   * @return z-axis value
   */
  public double getZ(){
    return this.dimensions[Z_DIM];
  }

  /**
   * Sets the x-axis value
   * @param value x-axis value to set
   */
  public void setX(double value){
    this.dimensions[X_DIM] = value;
  }

  /**
   * Sets the y-axis value
   * @param value y-axis value to set
   */
  public void setY(double value){
    this.dimensions[Y_DIM] = value;
  }

  /**
   * Sets the z-axis value
   * @param value z-axis value to set
   */
  public void setZ(double value){
    this.dimensions[Z_DIM] = value;
  }

  /**
   * Gets the index used internally to mark the x-axis dimension
   * @return The index
   */
  public static int getXIndex() {
    return X_DIM;
  }

  /**
   * Gets the index used internally to mark the y-axis dimension
   * @return The index
   */
  public static int getYIndex() {
    return Y_DIM;
  }

  /**
   * Gets the index used internally to mark the z-axis dimension
   * @return The index
   */
  public static int getZIndex() {
    return Z_DIM;
  }
}