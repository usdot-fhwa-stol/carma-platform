package gov.dot.fhwa.saxton.glidepath.ead.trajectorytree;

import java.util.List;

/**
 * An interface defining the functions needed for a class which can find a path through a graph
 */
public interface ITreeSolver {
  /**
   * Function returns the path from a starting node to the goal as defined in the ICostModel
   * The neighbors of nodes on the graph are determined by the INeighborCalculator
   *
   * @param start The starting node
   * @param costModel The cost model
   * @param neighborCalculator The neighbor calculator
   * @return The path from start to goal as a list of nodes
   */
  List<Node> solve(Node start, ICostModel costModel, INeighborCalculator neighborCalculator);
}
