package gov.dot.fhwa.saxton.glidepath.ead.trajectorytree;

/**
 * Interface defining the functions needed for a cost model to be used with an ITreeSolver
 */
public interface ICostModel {
  /**
   * Calculates the cost between two nodes
   * @param n1 First node
   * @param n2 Second node
   * @return The cost
   */
  double cost(Node n1, Node n2);

  /**
   * Calculates the heuristic value of node assuming some goal.
   * A heuristic is considered admissible if it is always <= actual cost of reaching the goal
   * @param n1 The node
   * @return the heuristic value
   */
  double heuristic(Node n1);

  /**
   * Allows specification of a set of tolerances on the various Node dimensions to facilitate approximate goal comparison
   * @param tolerances A node containing allowable tolerances in each dimension
   */
  void setTolerances(Node tolerances);

  /**
   * Defines a node as the goal (primarily used for the heuristic calculation).
   * @param goal - the goal node
   */
  void setGoal(Node goal);

  /**
   * Returns true if the provided node is the equal to the goal or within specified tolerances of the goal
   * @param n The node
   * @return True if n equals goal
   *
   * @apiNote If setTolerances has not been called then the comparison must satisfy equality
   */
  boolean isGoal(Node n);

  /**
   * Returns true if n is legal but clearly not part of a practical solution for the problem at hand
   * @param n The node
   * @return true if N should be discarded from consideration
   */
  boolean isUnusable(Node n);
}
