package gov.dot.fhwa.saxton.glidepath.ead.trajectorytree;

import gov.dot.fhwa.saxton.glidepath.asd.IntersectionData;

import java.util.List;

/**
 * Interface defining the functions needed calculating node neighbors for use with ITreeSolver
 */
public interface INeighborCalculator {

  /**
   * Provides all of the initial data necessary to identify neighbors in an EAD environment
   * @param intersections - list of known intersections, sorted in order from nearest to farthest
   * @param numIntersections - the number of intersections to consider in this solution
   * @param timeIncrement - increment between adjacent time points in the grid, sec
   * @param speedIncrement - increment between adjacent speed points in the grid, m/s
   */
  void initialize(List<IntersectionData> intersections, int numIntersections, double timeIncrement,
                  double speedIncrement);

  /**
   * Gets a list of neighbors to the provided node
   * @param node The node
   * @return List of node's neighbors
   */
  List<Node> neighbors(Node node);
}
