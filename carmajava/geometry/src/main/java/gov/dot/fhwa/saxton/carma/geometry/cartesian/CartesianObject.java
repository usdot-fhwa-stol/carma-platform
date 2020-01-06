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

import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;
import org.ros.rosjava_geometry.Transform;

import org.ros.rosjava_geometry.Vector3;

/**
 * An object in n-dimensional cartesian space defined by a point cloud.
 * The bounds of the object are calculated on construction and can be used for intersection checking
 */
public class CartesianObject implements CartesianElement {

  public static final int MIN_BOUND_IDX = 0;
  public static final int MAX_BOUND_IDX = 1;
  protected double[][] bounds; // 2 cols as every dim has a min and max value
  protected double[][] minMaxCoordinates; // Transpose of bounds #cols = #dimension such that the first row is (min,min,..., min) and the second row is (max,max,...,max)
  protected double[] size;
  protected Point centroidOfBounds;
  protected Point centroidOfCloud;
  protected final int numDimensions;
  protected List<? extends Point> pointCloud;

  /**
   * Constructor which defines a cartesian object by the provided point cloud
   * All provided points must be of the same dimension
   * @param pointCloud The point cloud which defines this object
   * @throws IllegalArgumentException Thrown if the provided point cloud has points of varying dimensions
   */
  public CartesianObject(List<? extends Point> pointCloud) throws IllegalArgumentException {
    this.validateInput(pointCloud);
    this.numDimensions = pointCloud.get(0).getNumDimensions();
    this.pointCloud = pointCloud;
  }

  /**
   * Helper function to validate the input to the constuctor
   * @param points The point cloud to validate
   * @throws IllegalArgumentException Thrown if the provided point cloud has points of varying dimensions
   */
  protected void validateInput(List<? extends Point> points) throws IllegalArgumentException {
    if (points.size() <= 0) {
      throw new IllegalArgumentException("Empty list of points provided to CartesianObject constructor");
    }
    int expectedDimensions = 0;
    boolean firstPoint = true;
    for (Point p : points) {
      if (firstPoint) {
        expectedDimensions = p.getNumDimensions();
        firstPoint = false;
      } else if (p.getNumDimensions() != expectedDimensions) {
        throw new IllegalArgumentException("Inconsistent dimensions in list of points provided to CartesianObject constructor");
      }
    }
  }

  /**
   * Calculates the centroid of this object's point cloud
   * Assumes that validateInput() has already been called
   */
  protected void calculateCentroidOfCloud() {
    Vector centroidValues = new Vector(new Point(new double[numDimensions]));
    for (int i = 0; i < pointCloud.size(); i++) {
      for (int j = 0; j < numDimensions; j++) {
        centroidValues.setDim(j, centroidValues.getDim(j) + pointCloud.get(i).getDim(j));
      }
    }
    centroidOfCloud = centroidValues.scalarMultiply(1.0 / pointCloud.size()).toPoint();
  }

  /**
   * Calculates the centroid of this object's bounds
   * Assumes calculateBounds() has already been called
   */
  protected void calculateCentroidOfBounds() {
    if (bounds == null) {
      calculateBounds();
    }
    double[] centroidValues = new double[numDimensions];
    for (int i = 0; i < numDimensions; i++) {
      centroidValues[i] = (bounds[i][MIN_BOUND_IDX] + bounds[i][MAX_BOUND_IDX]) / 2;
    }
    centroidOfBounds = new Point(centroidValues);
  }

  /**
   * Calculates the bounds of the provided point cloud
   */
  protected void calculateBounds() {
    int dims = this.getNumDimensions();
    bounds = new double[dims][2];
    minMaxCoordinates = new double[2][dims];
    boolean firstPoint = true;
    for (Point p : pointCloud) {
      for (int i = 0; i < dims; i++) {
        if (firstPoint) {
          bounds[i][MIN_BOUND_IDX] = p.getDim(i);
          bounds[i][MAX_BOUND_IDX] = p.getDim(i);
          minMaxCoordinates[MIN_BOUND_IDX][i] = bounds[i][MIN_BOUND_IDX];
          minMaxCoordinates[MAX_BOUND_IDX][i] = bounds[i][MAX_BOUND_IDX];
        } else if (p.getDim(i) < bounds[i][MIN_BOUND_IDX]) {
          bounds[i][MIN_BOUND_IDX] = p.getDim(i);
          minMaxCoordinates[MIN_BOUND_IDX][i] = bounds[i][MIN_BOUND_IDX];
        } else if (p.getDim(i) > bounds[i][MAX_BOUND_IDX]) {
          bounds[i][MAX_BOUND_IDX] = p.getDim(i);
          minMaxCoordinates[MAX_BOUND_IDX][i] = bounds[i][MAX_BOUND_IDX];
        }
      }
      firstPoint = false;
    }
  }

  /**
   * Get's the set of points originally used to define this object
   * @return list of points of the same dimension
   */
  public List<? extends Point> getPointCloud() {
    return pointCloud;
  }

  /**
   * Gets the index used for minimum bounds in the 2d bounds array
   * @return index
   */
  public int getMinBoundIndx() {
    return MIN_BOUND_IDX;
  }

  /**
   * Gets the index used for maximum bounds in the 2d bounds array
   * @return index
   */
  public int getMaxBoundIndx() {
    return MAX_BOUND_IDX;
  }

  /**
   * Gets the bounds of this object
   * @return A 2d array where the rows are the dimension and the columns are the min/max values
   */
  public double[][] getBounds() {
    if (bounds == null) {
      calculateBounds();
    }
    return bounds;
  }

  /**
   * Gets the min and max coordinates of this object bounds
   * This returns a transpose of the getBounds() function
   * 
   * @return A 2d array where the rows are the min/max and the columns are the dimensions values
   */
  public double[][] getMinMaxCoordinates() {
    if (minMaxCoordinates == null) {
      calculateBounds();
    }
    return minMaxCoordinates;
  }

  /**
   * Calculates the size of the provided point cloud
   * Assumes that validateInput() has already been called
   * @param points A list of points assumed to be of the same dimension
   */
  protected void calculateSize() {
    size = new double[bounds.length];

    for (int i = 0; i < bounds.length; i++) {
      size[i] = bounds[i][MAX_BOUND_IDX] - bounds[i][MIN_BOUND_IDX];
    }
  }

  /**
   * Gets the size of this object
   * @return A array each entry corresponding to the size of the object bounds along the relevant dimension
   */
  public double[] getSize() {
    if (size == null) {
      calculateSize();
    }
    return size;
  }

  /**
   * Gets the centroid of this object's bounds
   * @return the bounds centroid
   */
  public Point getCentroidOfBounds() {
    if (centroidOfBounds == null) {
      calculateCentroidOfBounds();
    }
    return centroidOfBounds;
  }

  /**
   * Gets the centroid of this object's original point cloud
   * @return the point cloud centroid
   */
  public Point getCentroidOfCloud() {
    if (centroidOfCloud == null) {
      calculateCentroidOfCloud();
    }
    return centroidOfCloud;
  }

  /**
   * Returns true if the provided dimensions are all larger than this object's corresponding measurements
   * Only matching dimensions will be checked. 
   * Dimensions are assumed to be positive
   * Mismatched dimensions will return true as this object has 0 size on that axis
   * 
   * @param dimensions An array of dimensions to compare. All dimensions should be positive
   * @param multiplier A multiplier to apply to this objects size when doing comparison
   * 
   * @return True if would fit inside
   */
  public boolean canFitInside(double[] dimensions, double multiplier) {
    int numDims = dimensions.length < size.length ? dimensions.length : size.length;
    for (int i = 0; i < numDims; i++) {
      if (dimensions[i] < size[i] * multiplier) {
        return false;
      }
    }
    return true;
  }

  /**
   * Transforms this object into a new frame by applying the provided transform to all points
   * This transform only works on 3D objects
   * 
   * @param transform The transform to apply
   * 
   * @return a new CartesianObject with the transform applied
   */
  public CartesianObject transform(Transform transform) throws IllegalArgumentException {
    if (numDimensions != 3) {
      throw new IllegalArgumentException("Cannot transform a non 3D object");
    }
    List<Point3D> newPoints = new LinkedList<>();
    for (Point p: pointCloud) {
      Vector3 vec = transform.apply(new Vector3(p.getDim(0), p.getDim(1), p.getDim(2)));
      newPoints.add(new Point3D(vec.getX(), vec.getY(), vec.getZ()));
    }

    return new CartesianObject(newPoints);
  }

  @Override public int getNumDimensions() {
    return numDimensions;
  }

  @Override public String toString() {
    double[][] minMaxArray = getMinMaxCoordinates();
    return this.getClass().getSimpleName() + " {" +"min " + Arrays.toString(minMaxArray[0]) + ", max " + Arrays.toString(minMaxArray[1]) + "}";
  }
}
