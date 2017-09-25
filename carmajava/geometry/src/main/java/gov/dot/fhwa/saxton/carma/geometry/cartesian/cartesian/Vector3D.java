package gov.dot.fhwa.saxton.carma.geometry.cartesian.cartesian;

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

  // cross product is only defined for 3D vectors
  public Vector3D cross(Vector3D v2) throws IllegalArgumentException {
    double x = this.getDim(1) * v2.getDim(2) - this.getDim(2) * v2.getDim(1);
    double y = this.getDim(2) * v2.getDim(0) - this.getDim(0) * v2.getDim(2);
    double z = this.getDim(0) * v2.getDim(1) - this.getDim(1) * v2.getDim(0);

    return new Vector3D(new Point3D(x,y,z));
  }

  public Point3D toPoint3D() {
    Point p = this.toPoint();
    return new Point3D(p.getDim(0), p.getDim(1), p.getDim(2));
  }
}
