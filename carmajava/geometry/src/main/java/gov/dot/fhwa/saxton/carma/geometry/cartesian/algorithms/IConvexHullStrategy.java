package gov.dot.fhwa.saxton.carma.geometry.cartesian.algorithms;

import gov.dot.fhwa.saxton.carma.geometry.cartesian.Point;

import java.util.List;

/**
 * Created by mcconnelms on 10/4/17.
 */
public interface IConvexHullStrategy {
  List<Point> calculateConvexHull(List<Point> points);
}
