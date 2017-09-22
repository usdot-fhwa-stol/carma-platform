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
 * A representation of a point in 3-dimensional space.
 */
public class Point3D extends Point{

  private final int X_DIM = 0;
  private final int Y_DIM = 1;
  private final int Z_DIM = 2;

  public Point3D(double x, double y, double z){
    super(3, 0); // Ensure there is space for x,y,z
    this.dimensions[X_DIM] = x;
    this.dimensions[Y_DIM] = y;
    this.dimensions[Z_DIM] = z;
  }

  public double getX(){
    return this.dimensions[X_DIM];
  }

  public double getY(){
    return this.dimensions[Y_DIM];
  }

  public double getZ(){
    return this.dimensions[Y_DIM];
  }

  public void setX(double value){
    this.dimensions[X_DIM] = value;
  }

  public void setY(double value){
    this.dimensions[Y_DIM] = value;
  }

  public void setZ(double value){
    this.dimensions[Z_DIM] = value;
  }
}