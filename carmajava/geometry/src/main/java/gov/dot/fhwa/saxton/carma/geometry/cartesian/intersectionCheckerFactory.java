package gov.dot.fhwa.saxton.carma.geometry.cartesian;

/**
 * Created by mcconnelms on 10/4/17.
 */
public class intersectionCheckerFactory {
  public IIntersectionChecker makeIntersectionChecker(String method) {
    switch (method) {
      case "AABB":
        return new AxisAlignedBoundingBox();
      default:
        return new AxisAlignedBoundingBox();
    }
  }
}
