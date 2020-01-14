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

import org.ros.rosjava_geometry.Vector3;

/**
 * A representation of a vector in 3-dimensional space.
 * While a vector can be calculated form a head and tail point,
 * it is always represented only as the distances between these two points.
 */
public class Vector3D extends Vector {

  /**
   * Constructs vector from x,y,z values
   * @param x x-axis value
   * @param y y-axis value
   * @param z z-axis value
   */
  public Vector3D(double x, double y, double z) {
    super(new Point3D(x,y,z));
  }
  /**
   * Constructor defines a vector from the tail point to the head point
   * @param tail The tail of the vector. Must be the same dimension as the head
   * @param head The head of the vector (the arrow when drawing)
   */
  public Vector3D(Point3D tail, Point3D head) {
    super(tail, head);
  }

  /**
   * Constructor defines a vector from an origin to the provided head point.
   * The origin is in the frame used to define the provided head point
   * @param head The head point of the vector. Point is deep copied.
   */
  public Vector3D(Point3D head){
    super(head);
  }

  /**
   * Defines a vector as a deep copy of the provided vector.
   * @param vec The vector to copy
   */
  public Vector3D(Vector3D vec){
    super(vec);
  }

  /**
   * Calculates the cross product of this vector and the provided vector
   * Cross product is only defined for 3d space
   * @param v2 The vector to apply
   * @return The vector resulting from the cross product
   */
  public Vector3D cross(Vector3D v2) {
    double x = this.getY() * v2.getZ() - this.getZ() * v2.getY();
    double y = this.getZ() * v2.getX() - this.getX() * v2.getZ();
    double z = this.getX() * v2.getY() - this.getY() * v2.getX();

    return new Vector3D(new Point3D(x,y,z));
  }

  /**
   * Calculates the determinant of a 3x3 square matrix defined by 3 col vectors
   * @param c1 First column vector
   * @param c2 Second column vector
   * @param c3 Third column vector
   * @return The determinant
   */
  public static double get3by3Determinant(Vector3D c1, Vector3D c2, Vector3D c3) {
    final double det1 = c2.getY()*c3.getZ() - c3.getY()*c2.getZ();
    final double det2 = c1.getY()*c3.getZ() - c3.getY()*c1.getZ();
    final double det3 = c1.getY()*c2.getZ() - c2.getY()*c1.getZ();
    return c1.getX()*det1 - c2.getX()*det2 + c3.getX()*det3;
  }

  /**
   * Gets the x-axis value
   * @return x-axis value
   */
  public double getX() {
    return headPoint_.getDim(Point3D.getXIndex());
  }

  /**
   * Gets the y-axis value
   * @return y-axis value
   */
  public double getY() {
    return headPoint_.getDim(Point3D.getYIndex());
  }

  /**
   * Gets the z-axis value
   * @return z-axis value
   */
  public double getZ() {
    return headPoint_.getDim(Point3D.getZIndex());
  }

  /**
   * Helper function to convert a Vector to a 3D vector.
   * Any dimensions not set which are < 3 are set to 0
   * Any dimensions > 3 are dropped
   * @param v The vector to convert to a Vector3D
   * @return A new vector 3d
   */
  public static Vector3D fromVector(Vector v) {
    Point3D p = new Point3D(0,0,0);
    for (int i = 0; i < 3; i++) {
      if (i < v.getNumDimensions()) {
        p.setDim(i, v.getDim(i));
      } else {
        break;
      }
    }

    return new Vector3D(p);
  }

  /**
   * Helper function to convert a rosjava Vector3 to a 3D vector.
   * @param v The vector to convert to a Vector3D
   * @return A new vector 3d
   */
  public static Vector3D fromVector(Vector3 v) {
    return new Vector3D(new Point3D(v.getX(),v.getY(),v.getZ()));
  }
}
