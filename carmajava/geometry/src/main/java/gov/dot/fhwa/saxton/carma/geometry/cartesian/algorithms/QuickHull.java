package gov.dot.fhwa.saxton.carma.geometry.cartesian.algorithms;

import gov.dot.fhwa.saxton.carma.geometry.cartesian.Point;

import java.util.LinkedList;
import java.util.List;

/**
 * Created by mcconnelms on 10/4/17.
 */
public class QuickHull implements IConvexHullStrategy {
  @Override public List<Point> calculateConvexHull(List<Point> points) {
    List<Point> hull;
    if (points.size() < points.get(0).getNumDimensions() + 1) {
      return new LinkedList<>(points); // TODO make this deep copy
    } // TODO any input validation needed

    List<Point> boundingPoints = findExtremePoints(points);
    Input = a set S of n points
    Assume that there are at least 2 points in the input set S of points
    QuickHull (S)
    {
      // Find convex hull from the set S of n points
      Convex Hull := {}
      Find left and right most points, say A & B, and add A & B to convex hull
      Segment AB divides the remaining (n-2) points into 2 groups S1 and S2
      where S1 are points in S that are on the right side of the oriented line from A to B,
      and S2 are points in S that are on the right side of the oriented line from B to A
      FindHull (S1, A, B)
      FindHull (S2, B, A)
    }
    FindHull (Sk, P, Q)
    {
      // Find points on convex hull from the set Sk of points
      // that are on the right side of the oriented line from P to Q
      If Sk has no point, then return.
      From the given set of points in Sk, find farthest point, say C, from segment PQ
      Add point C to convex hull at the location between P and Q
      Three points P, Q, and C partition the remaining points of Sk into 3 subsets: S0, S1, and S2
      where S0 are points inside triangle PCQ, S1 are points on the right side of the oriented
      line from  P to C, and S2 are points on the right side of the oriented line from C to Q.
      FindHull(S1, P, C)
      FindHull(S2, C, Q)
    }
    Output = Convex Hull
    return null;
  }

  protected List<Point> findExtremePoints(List<Point> points) {
    final int MIN_BOUND_IDX = 0;
    final int MAX_BOUND_IDX = 1;
    int dims = points.get(0).getNumDimensions();
    Point[][] boundingPoints = new Point[dims][2];
    double[][] bounds = new double[dims][2];

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



}
