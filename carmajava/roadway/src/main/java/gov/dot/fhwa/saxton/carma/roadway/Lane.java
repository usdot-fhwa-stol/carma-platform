package gov.dot.fhwa.saxton.carma.roadway;

import gov.dot.fhwa.saxton.carma.geometry.cartesian.Point3D;

import java.util.List;

/**
 * Geometric representation of a lane
 */
public class Lane {
  List<LaneSegment> segments;
  boolean inLane(Point3D point) {
    return false;
  }

  boolean inLane(IObstacle obstacle) {
    return false;
  }

  double length() {
    double sum = 0;
    for (LaneSegment seg : segments) {
      sum += seg.length();
    }

    return sum;
  }
}
