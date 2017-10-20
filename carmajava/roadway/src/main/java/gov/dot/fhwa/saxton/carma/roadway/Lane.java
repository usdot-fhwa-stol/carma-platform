package gov.dot.fhwa.saxton.carma.roadway;

import gov.dot.fhwa.saxton.carma.geometry.cartesian.Point3D;

import java.util.List;

/**
 * Geometric representation of a lane
 */
public class Lane {
  List<LaneSegment> segments;

  /**
   * TODO Implement
   * True if the specified point can be considered in the lane
   * @param point point to compare
   * @return true if in lane
   */
  boolean inLane(Point3D point) {
    return false;
  }

  /**
   * TODO Implement
   * True if the specified obstacle can be considered in the lane
   * @param obstacle obstacle to compare
   * @return true if in lane
   */
  boolean inLane(IObstacle obstacle) {
    return false;
  }

  /**
   * Returns the length of a lane
   * @return length in meters
   */
  double length() {
    double sum = 0;
    for (LaneSegment seg : segments) {
      sum += seg.length();
    }

    return sum;
  }
}
