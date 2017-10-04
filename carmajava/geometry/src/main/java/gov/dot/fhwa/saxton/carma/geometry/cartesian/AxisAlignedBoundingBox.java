package gov.dot.fhwa.saxton.carma.geometry.cartesian;

/**
 * Created by mcconnelms on 10/4/17.
 */
public class AxisAlignedBoundingBox implements IIntersectionChecker {
  @Override public boolean intersects(CartesianObject obj, Point p) throws IllegalArgumentException {
    if (obj.getNumDimensions() != p.getNumDimensions())
      throw new IllegalArgumentException("Cannot check the intersection of CartesianElements with different dimensions");
    final double[][] objBounds = obj.getBounds();
    final int minIdx = obj.getMinBoundIndx();
    final int maxIdx = obj.getMaxBoundIndx();

    for (int i = 0; i < p.getNumDimensions(); i++) {
      if (!(objBounds[i][minIdx] < p.getDim(i) && p.getDim(i) < objBounds[i][maxIdx])) {
        return false;
      }
    }

    return true;
  }

  @Override public boolean intersects(CartesianObject obj, LineSegment seg) throws IllegalArgumentException {
    return false;
  }

  @Override public boolean intersects(CartesianObject obj, Vector vec) throws IllegalArgumentException {
    return intersects(obj, new LineSegment(new Point(vec.getNumDimensions(), 0.0), vec.toPoint()));
  }

  @Override public boolean intersects(CartesianObject obj, CartesianObject obj2) throws IllegalArgumentException {
    if (obj.getNumDimensions() != obj2.getNumDimensions())
      throw new IllegalArgumentException("Cannot check the intersection of CartesianElements with different dimensions");
    final double[][] objBounds = obj.getBounds();
    final int minIdx = obj.getMinBoundIndx();
    final int maxIdx = obj.getMaxBoundIndx();

    for (int i = 0; i < obj.getNumDimensions(); i++) {
      if (!(objBounds[i][minIdx] < p.getDim(i) && p.getDim(i) < objBounds[i][maxIdx])) {
        return false;
      }
    }

    return true;
    return false;
  }
}
