package gov.dot.fhwa.saxton.carma.geometry.cartesian;

import java.util.List;

/**
 * An object in n-dimensional cartesian space defined by a point cloud. The bounds of the object are calculated and can be used for intersection checking
 */
public class CartesianObject implements CartesianElement {

  protected final int MIN_BOUND_IDX = 0;
  protected final int MAX_BOUND_IDX = 1;
  protected double[][] bounds; // 2 rows as every dim has a min and max value
  protected Point centroidOfBounds;
  protected Point centroidOfHull;
  protected int numDimensions;
  protected List<Point> pointCloud;

  public CartesianObject(List<Point> pointCloud) throws IllegalArgumentException {
    this.validateInput(pointCloud);
    this.numDimensions = pointCloud.get(0).getNumDimensions();
    this.calculateBounds(pointCloud);
    this.calculateCentroidOfBounds();
    this.pointCloud = pointCloud;
    this.calculateCentroidOfCloud();
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

  protected void calculateCentroidOfCloud() {
    Vector centroidValues = new Vector(new Point(getNumDimensions(), 0));
    for (int i = 0; i < pointCloud.size(); i++) {
      centroidValues.setDim(i, centroidValues.getDim(i) + pointCloud.get(i).getDim(i));
    }
    centroidOfHull = centroidValues.scalarMultiply(1.0 / pointCloud.size()).toPoint();
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
        } else if (p.getDim(i) < bounds[i][MIN_BOUND_IDX]) {
          bounds[i][MIN_BOUND_IDX] = p.getDim(i);
        } else if (p.getDim(i) > bounds[i][MAX_BOUND_IDX]) {
          bounds[i][MAX_BOUND_IDX] = p.getDim(i);
        }
      }
      firstPoint = false;
    }
  }

  @Override public int getNumDimensions() {
    return numDimensions;
  }

  public List<Point> getPointCloud() {
    return pointCloud;
  }

  public int getMinBoundIndx() {
    return MIN_BOUND_IDX;
  }

  public int getMaxBoundIndx() {
    return MAX_BOUND_IDX;
  }

  public double[][] getBounds() {
    return bounds;
  }

  public Point getCentroidOfBounds() {
    return centroidOfBounds;
  }

  public Point getCentroidOfHull() {
    return centroidOfHull;
  }
}
