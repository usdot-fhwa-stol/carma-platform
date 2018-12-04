/*
 * Copyright (C) 2018 LEIDOS.
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

package gov.dot.fhwa.saxton.carma.signal_plugin.ead.trajectorytree;

import gov.dot.fhwa.saxton.carma.signal_plugin.logger.ILogger;
import gov.dot.fhwa.saxton.carma.signal_plugin.logger.LoggerManager;

import java.util.*;

import org.apache.commons.lang.mutable.MutableDouble;

/**
 * Implements the AStar algorithm which can operate on nodes with distance, time, speed states
 * The cost, heuristic, and goal is provided by the ICostModel
 * The neighbors of a node is provided by the INeighborCalculator
 * This can function as a Dijkstra solver if the ICostModel always returns a heuristic of 0
 */
public class ANAStarSolver implements ITreeSolver {

  protected static final ILogger log_ = LoggerManager.getLogger(AStarSolver.class);
  protected long maxPlanningTimeMS = 500;
  public static long iterationCount = 0;

  @Override
  public List<Node> solve(Node start, ICostModel costModel, INeighborCalculator neighborCalculator) {

    // List of optimal parent from each node
    // This map is used to extract the optimal path once the goal is found
    final Map<Node,Node> cameFrom = new HashMap<>(1500);

    // List of nodes which will no longer be explored as their cost is guaranteed to be greater then G
    final Set<Node> closedSet = new HashSet<>(1500);

    // Each node's actual cost to reach from the start node
    final Map<Node, Double> gScore = new HashMap<>(1500);

    // Each node's heuristic cost to reach from the start node
    final Map<Node, Double> hScore = new HashMap<>(1500);

    // Expected e-score cost to reach goal from start through each node
    // eScore determines order of node expansion
    // eScore = (G - gScore) / hScore
    final Map<Node, Double> eScore = new HashMap<>(1500);

    // Queue of discovered nodes which still need to be evaluated
    // The queue is ordered by eScore
    // No assumption is made about the ordering of nodes with equal eScore
    // Higher eScore means higher priority
    PriorityQueue<Node> openSetQueue = new PriorityQueue<>(1500, new Comparator<Node>() {
      @Override public int compare(Node n1, Node n2) {
        return eScore.get(n1) > eScore.get(n2) ? -1 : 1;
      }
    });


    // Initialize values
    MutableDouble G = new MutableDouble(Double.POSITIVE_INFINITY);
    MutableDouble E = new MutableDouble(Double.POSITIVE_INFINITY);

    // Cost of going from start to start is zero.
    gScore.put(start, 0.0);
    hScore.put(start, costModel.heuristic(start));
    openSetQueue.add(start);

    List<Node> bestPath = new LinkedList<>();

    iterationCount = 0;

    long endTime = System.currentTimeMillis() + maxPlanningTimeMS;
    if (endTime < 0) {
      // Overflow has occurred which means we should use Long.MAX_VALUE
      endTime = Long.MAX_VALUE;
    }
    while(!openSetQueue.isEmpty()) {
      iterationCount++;
      List<Node> result = improveSolution(start, costModel, neighborCalculator, G, E, openSetQueue, gScore, eScore, hScore, cameFrom, closedSet, endTime);

      // If the result is not empty then it contains a better path
      if (!result.isEmpty()) {
        bestPath = result;
      }

      // If more than the allowable time has elapsed break
      if (System.currentTimeMillis() > endTime) {
        break;
      }
      // TODO could print here

      // Update eScores in openSet with new G and prune the open set

      List<Node> nodeToUpdate = new ArrayList<>(openSetQueue.size());
      for (Node n: openSetQueue) {
        final double gScoreOfNode = gScore.get(n);
        final double hScoreOfNode = hScore.get(n);

        if (gScoreOfNode + hScoreOfNode >= G.doubleValue()) {
          closedSet.add(n);
          //eScore.put(n, Double.NEGATIVE_INFINITY);
          continue;
        }

        final double newEScore = (G.doubleValue() - gScoreOfNode) / hScoreOfNode;
        eScore.put(n, newEScore);
        nodeToUpdate.add(n);
      }

      // // Easiest way to update eScore values is rebuild the priority queue
      openSetQueue = new PriorityQueue<>(1500, new Comparator<Node>() {
        @Override public int compare(Node n1, Node n2) {
          return eScore.get(n1) > eScore.get(n2) ? -1 : 1;
        }
      });

      openSetQueue.addAll(nodeToUpdate);

    }

    //System.out.println("IterationCount: " + iterationCount);
    return bestPath; // Return empty list if no path exists
  }






  /**
   * @param start
   * @param costModel
   * @param neighborCalculator
   * @param G
   * @return
   */

  protected List<Node> improveSolution(Node start, ICostModel costModel, INeighborCalculator neighborCalculator,
   MutableDouble G, MutableDouble E, PriorityQueue<Node> openSetQueue,
    Map<Node, Double> gScore, Map<Node, Double> eScore, Map<Node, Double> hScore,
    Map<Node,Node> cameFrom,
    Set<Node> closedSet,
    long endTime) {


    long visitedNodes = 0;
    boolean firstRun = G.doubleValue() == Double.POSITIVE_INFINITY;



    // Begin search
    while (!openSetQueue.isEmpty() && (firstRun || (System.currentTimeMillis() < endTime) ) ) {
      Node current = openSetQueue.poll(); // Retrieve and remove the next node on the queue
      visitedNodes++;
      
      
      if (closedSet.contains(current)) {
        continue;
      }

      double gScoreOfNode = gScore.get(current);
      double hScoreOfNode = hScore.get(current);
      double eScoreOfNode = (G.doubleValue() - gScoreOfNode) / hScoreOfNode;
      eScore.put(current, eScoreOfNode);

      if (eScoreOfNode < E.doubleValue()) {
        E.setValue(eScoreOfNode);
      }

      // Check if this node is the goal
      if (costModel.isGoal(current)) {
        log_.info("EAD", "Found our goal with node " + current.toString());
        log_.info("EAD", "Ending sizes: closedSet=" + closedSet.size() + ", cameFrom=" +
                    cameFrom.size() + ", gScore=" + gScore.size() + ", openSetQueue=" + openSetQueue.size());
        log_.debug("EAD", "We have visited " + visitedNodes + " nodes to find the solution");

        G.setValue(gScore.get(current));

        //System.out.println("We have visited " + visitedNodes + " nodes to find the solution");
        List<Node> path = rebuildPath(cameFrom, current); // Get path
        //System.out.println("The total cost is " + fScore.get(path.get(path.size() - 1)));
        return path; 
      }

      //if this node is unusable then toss it out and move on
      if (costModel.isUnusable(current) || (gScoreOfNode + hScoreOfNode >= G.doubleValue())) {
        //log_.debug("EAD", "Skipping unusable node " + current.toString());
        closedSet.add(current);
        continue;
      }

      // Iterate over the list of neighbors
      List<Node> neighbors = neighborCalculator.neighbors(current);
      for (Node neighbor : neighbors) {
        if (closedSet.contains(neighbor)) {    // Ignore the neighbor which is already visited.
          //log_.debug("EAD", "Neighbor " + neighbor.toString() + " is already in the closed set.");
          continue;
        }

        // Calculate cost to neighbor
        double cost = costModel.cost(current, neighbor);

        double tentativeGScore = gScoreOfNode + cost;
        Double oldGScore = gScore.get(neighbor);

        if (oldGScore == null || tentativeGScore < oldGScore) {
          gScore.put(neighbor, tentativeGScore);
          cameFrom.put(neighbor, current);

          Double neighborHScore = hScore.get(neighbor);
          
          if (neighborHScore == null) {
            neighborHScore = costModel.heuristic(neighbor);
            hScore.put(neighbor, neighborHScore);
          }

          if (tentativeGScore + neighborHScore < G.doubleValue()) {
            double neighborEScore = (G.doubleValue() - tentativeGScore) / neighborHScore;
            eScore.put(neighbor, neighborEScore);
            openSetQueue.add(neighbor);
          }
        }
      }
    }
    log_.info("EAD", "///// solve:  No solution found.");

    return new LinkedList<>(); // Return empty list if no path exists
  }

  public void setMaxPlanningTimeMS(long maxPlanningTimeMS) {
    this.maxPlanningTimeMS = maxPlanningTimeMS;
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
