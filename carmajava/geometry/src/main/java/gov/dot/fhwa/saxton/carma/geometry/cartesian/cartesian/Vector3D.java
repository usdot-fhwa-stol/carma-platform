package gov.dot.fhwa.saxton.carma.geometry.cartesian.cartesian;

import org.ros.rosjava_geometry.Vector3;

/**
 * Created by mcconnelms on 9/25/17.
 */
public class Vector3D extends Vector {

  public Vector3D(Point3D head, Point3D tail) {
    super(head, tail);
  }

  public Vector3D(Point3D head){
    super(head);
  }

  public Vector3D(Vector3D vec){
    super(vec);
  }

  public Vector3D(Vector3 vec) {
    super(new Vector3D(new Point3D(vec.getX(), vec.getY(), vec.getZ())));
  }

  // cross product is only defined for 3D vectors
  public Vector3D cross(Vector3D v2) {
    double x = this.getY() * v2.getZ() - this.getZ() * v2.getY();
    double y = this.getZ() * v2.getX() - this.getX() * v2.getZ();
    double z = this.getX() * v2.getY() - this.getY() * v2.getX();

    return new Vector3D(new Point3D(x,y,z));
  }

  public double getX() {
    return headPoint_.getDim(Point3D.getXIndex());
  }

  public double getY() {
    return headPoint_.getDim(Point3D.getYIndex());
  }

  public double getZ() {
    return headPoint_.getDim(Point3D.getZIndex());
  }

  public Vector3 toVector3() {
    return new Vector3(getX(), getY(), getZ());
  }
}
