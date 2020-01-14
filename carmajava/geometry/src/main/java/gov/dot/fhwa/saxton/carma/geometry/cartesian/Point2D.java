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
 * A representation of a point in 2-dimensional space.
 */
public class Point2D extends Point{

  private static final int X_DIM = 0;
  private static final int Y_DIM = 1;

  /**
   * Constructor defines a 2d point with the provided x and y values
   * @param x x-axis value
   * @param y y-axis value
   */
  public Point2D(double x, double y) {
    super(0, 0); //Ensure there is room for x and y
    this.dimensions[X_DIM] = x;
    this.dimensions[Y_DIM] = y;
  }

  /**
   * Constructor defines this point as a deep copy of the provided point
   * @param p The point to copy
   */
  public Point2D(Point2D p) {
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
}
