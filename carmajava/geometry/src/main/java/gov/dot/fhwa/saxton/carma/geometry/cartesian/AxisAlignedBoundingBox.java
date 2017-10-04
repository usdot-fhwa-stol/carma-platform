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

  @Override public boolean intersects(CartesianObject obj, CartesianObject obj2) throws IllegalArgumentException {
    if (obj.getNumDimensions() != obj2.getNumDimensions())
      throw new IllegalArgumentException("Cannot check the intersection of CartesianElements with different dimensions");
    final double[][] bounds1 = obj.getBounds();
    final double[][] bounds2 = obj2.getBounds();
    final int min1 = obj.getMinBoundIndx();
    final int max1 = obj.getMaxBoundIndx();
    final int min2 = obj2.getMaxBoundIndx();
    final int max2 = obj2.getMaxBoundIndx();


    for (int i = 0; i < obj.getNumDimensions(); i++) {
      if (Math.max(bounds1[i][min1], bounds2[i][min2]) > Math.min(bounds1[i][max1], bounds2[i][max2])) {
        return false;
      }
    }
    return true;
  }
}
