/*
 * TODO: Copyright (C) 2017 LEIDOS.
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
 * A point in a 3D cartesian coordinate system.
 * While this class is an extension of Point, it will only support 3D locations.
 */
public class Point3D {
  // These variables will be moved once Point is implemented
  private double x;
  private double y;
  private double z;

  public Point3D() {
    this.x = 0;
    this.y = 0;
    this.z = 0;
  }

  public Point3D(double x, double y, double z) {
    this.x = x;
    this.y = y;
    this.z = z;
  }

  public double getX() {
    return x;
  }

  public void setX(double x) {
    this.x = x;
  }

  public double getY() {
    return y;
  }

  public void setY(double y) {
    this.y = y;
  }

  public double getZ() {
    return z;
  }

  public void setZ(double z) {
    this.z = z;
  }

  public boolean almostEquals(Point3D p2, double delta) {
    Vector3 v1 = new Vector3(getX(), getY(), getZ());
    return v1.almostEquals(new Vector3(p2.getX(), p2.getY(), p2.getZ()), delta);
  }

  public Vector3 toVector3() {
    return new Vector3(x,y,z);
  }

  @Override public String toString() {
    return super.toString() + " (x,y,z): (" + x + ", " + y + ", " + z + ")";
  }
}
