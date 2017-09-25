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

/**
 * A representation of a point in 2-dimensional space.
 */
public class Point2D extends Point{

  private final int X_DIM = 0;
  private final int Y_DIM = 1;

  public Point2D(double x, double y) {
    super(2, 0); //Ensure there is room for x and y
    this.dimensions[X_DIM] = x;
    this.dimensions[Y_DIM] = y;
  }

  public Point2D(Point2D p) {
    super(p);
  }

  public double getX(){
    return this.dimensions[X_DIM];
  }

  public double getY(){
    return this.dimensions[Y_DIM];
  }

  public void setX(double value){
    this.dimensions[X_DIM] = value;
  }

  public void setY(double value){
    this.dimensions[Y_DIM] = value;
  }

  public int getXIndex() {
    return X_DIM;
  }

  public int getYIndex() {
    return Y_DIM;
  }
}
