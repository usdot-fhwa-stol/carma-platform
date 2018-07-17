package gov.dot.fhwa.saxton.glidepath.ead.trajectorytree;

import gov.dot.fhwa.saxton.glidepath.logger.ILogger;
import gov.dot.fhwa.saxton.glidepath.logger.LoggerManager;

import java.util.*;

/**
 * Implements the AStar algorithm which can operate on nodes with distance, time, speed states
 * The cost, heuristic, and goal is provided by the ICostModel
 * The neighbors of a node is provided by the INeighborCalculator
 * This can function as a Dijkstra solver if the ICostModel always returns a heuristic of 0
 */
public class AStarSolver implements ITreeSolver {

  protected static ILogger log_ = LoggerManager.getLogger(AStarSolver.class);

  @Override
  public List<Node> solve(Node start, ICostModel costModel, INeighborCalculator neighborCalculator) {
    // Set of nodes already visited
    final Map<Node, Boolean> closedSet = new HashMap<>(100);

    // List of optimal parent from each node
    // This map is used to extract the optimal path once the goal is found
    final Map<Node,Node> cameFrom = new HashMap<>(100);

    // Each node's actual cost to reach from the start node
    final Map<Node, Double> gScore = new HashMap<>(800);

    // Cost of going from start to start is zero.
    gScore.put(start, 0.0);

    // Expected cost to reach goal from start through each node
    // fScore determines order of node expansion
    // fScore = gScore + hScore
    final Map<Node, Double> fScore = new HashMap<>(800);

    // For start node fScore = hScore (the heuristic)
    fScore.put(start, costModel.heuristic(start));

    // Queue of discovered nodes which still need to be evaluated
    // The queue is ordered by fScore
    // No assumption is made about the ordering of nodes with equal fScore
    final PriorityQueue<Node> openSetQueue = new PriorityQueue<>(800, new Comparator<Node>() {
      @Override public int compare(Node n1, Node n2) {
        return fScore.get(n1) < fScore.get(n2) ? -1 : 1;
      }
    });

    openSetQueue.add(start); // Place start on queue

    // Begin search
    while (!openSetQueue.isEmpty()) {
      Node current = openSetQueue.poll(); // Retrieve and remove the next node on the queue
      closedSet.put(current, true); // Mark the node visited

      // Check if this node is the goal
      if (costModel.isGoal(current)) {
        log_.debug("EAD", "Found our goal with node " + current.toString());
        log_.debug("EAD","Ending sizes: closedSet=" + closedSet.size() + ", cameFrom=" +
                    cameFrom.size() + ", gScore=" + gScore.size() + ", openSetQueue=" + openSetQueue.size());
        return rebuildPath(cameFrom, current); // Get path
      }

      //if this node is unusable then toss it out and move on
      if (costModel.isUnusable(current)) {
        //log_.debug("EAD", "Skipping unusable node " + current.toString());
        continue;
      }

      // Iterate over the list of neighbors
      List<Node> neighbors = neighborCalculator.neighbors(current);
      for (Node neighbor : neighbors) {
        if (closedSet.containsKey(neighbor)) {    // Ignore the neighbor which is already visited.
          //log_.debug("EAD", "Neighbor " + neighbor.toString() + " is already in the closed set.");
          continue;
        }

        // Calculate tentative gScore of neighbor
        double tentative_gScore = gScore.get(current) + costModel.cost(current, neighbor);
        Double neighborGScore = gScore.get(neighbor); // null if neighbor is previously undiscovered node

        // If this neighbor is not a new node and the tentative gScore is worse than the current gScore
        if (neighborGScore != null && tentative_gScore >= neighborGScore) {
          //log_.debug("EAD", "Neighbor " + neighbor.toString() + " has unacceptable tentative_gScore: "
          //            + tentative_gScore);
          continue;    // This is not a better path.
        }

        // Best path so far. Record path
        cameFrom.put(neighbor, current); // Mark current node as the parent of this neighbor node
        gScore.put(neighbor, tentative_gScore);
        double hScore = costModel.heuristic(neighbor);
        fScore.put(neighbor, tentative_gScore + hScore);
        //log_.debug("EAD", "Completed computations for neighbor " + neighbor.toString() +
        //            ". gScore = " + tentative_gScore);

        if (neighborGScore == null) { // If there was no gScore for this node then it is a new node
          openSetQueue.add(neighbor); // Add the new node to the openSetQueue
          //log_.debug("EAD", "Adding neighbor to the openSetQueue. Queue size = " + openSetQueue.size());
        }
      }
    }
    log_.info("EAD", "solve:  No solution found.");
    return new LinkedList<>(); // Return empty list if no path exists
  }

  /**
   * Helper function walks the list of parents from the goal to start to determine the optimal path
   * The resulting goal->start path is flipped before being returned so that it is in order of start->goal
   * @param cameFrom Map of parents for all discovered nodes
   * @param current The current node (should be goal node)
   * @return The list of nodes which forms the optimal path from start->goal
   */
  protected List<Node> rebuildPath(Map<Node, Node> cameFrom, Node current) {
    List<Node> path = new LinkedList<>(Arrays.asList(current));
    while (cameFrom.containsKey(current)) {
      current = cameFrom.get(current);
      path.add(current);
    }
    Collections.reverse(path);
    return path;
  }
}
