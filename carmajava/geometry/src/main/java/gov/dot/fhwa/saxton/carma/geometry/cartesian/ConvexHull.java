package gov.dot.fhwa.saxton.carma.geometry.cartesian;

import java.util.List;

/**
 * Created by mcconnelms on 9/22/17.
 */
public class ConvexHull implements DimensionalObject {

  protected final int MIN_BOUND_IDX = 0;
  protected final int MAX_BOUND_IDX = 1;
  protected double[][] bounds; // 2 rows as every dim has a min and max value
  protected Point centroidOfBounds;
  protected int numDimensions;

  public ConvexHull(List<Point> points) throws IllegalArgumentException {
    this.validateInput(points);
    this.numDimensions = points.get(0).getNumDimensions();
    this.calculateBounds(points);
    this.calculateCentroidOfBounds();
    
  }

  protected void validateInput(List<Point> points) throws IllegalArgumentException {
    if (points.size() <= 0) {
      throw new IllegalArgumentException("Empty list of points provided to ConvexHull constructor");
    }
    int expectedDimensions = 0;
    boolean firstPoint = true;
    for (Point p : points) {
      if (firstPoint) {
        expectedDimensions = p.getNumDimensions();
        firstPoint = false;
      } else if (p.getNumDimensions() != expectedDimensions) {
        throw new IllegalArgumentException("Inconsistent dimensions in list of points provided to ConvexHull constructor");
      }
    }
  }

  protected void calculateCentroidOfBounds() {
    double[] centroidValues = new double[bounds.length];
    for (int i = 0; i < bounds.length; i++) {
      centroidValues[i] = (bounds[i][MIN_BOUND_IDX] + bounds[i][MAX_BOUND_IDX]) / 2;
    }
    centroidOfBounds = new Point(centroidValues);
  }

  protected void calculateBounds(List<Point> points) {
    int dims = this.getNumDimensions();
    bounds = new double[dims][2];

    boolean firstPoint = true;
    for (Point p : points) {
      for (int i = 0; i < p.getNumDimensions(); i++) {
        if (firstPoint) {
          bounds[i][MIN_BOUND_IDX] = p.getDim(i);
          bounds[i][MAX_BOUND_IDX] = p.getDim(i);
          firstPoint = false;
        } else if (p.getDim(i) < bounds[i][MAX_BOUND_IDX]) {
          bounds[i][MAX_BOUND_IDX] = p.getDim(i);
        } else if (p.getDim(i) > bounds[i][MAX_BOUND_IDX]) {
          bounds[i][MAX_BOUND_IDX] = p.getDim(i);
        }
      }
    }
  }

  @Override public int getNumDimensions() {
    return numDimensions;
  }
}
