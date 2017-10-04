package gov.dot.fhwa.saxton.carma.geometry.cartesian;

/**
 * Created by mcconnelms on 10/4/17.
 */
public interface IIntersectionChecker {
  boolean intersects(CartesianObject obj, Point p) throws IllegalArgumentException ;
  boolean intersects(CartesianObject obj, LineSegment seg) throws IllegalArgumentException ;
  boolean intersects(CartesianObject obj, Vector vec) throws IllegalArgumentException ;
  boolean intersects(CartesianObject obj, CartesianObject obj2) throws IllegalArgumentException ;
}
