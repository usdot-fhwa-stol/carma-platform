package gov.dot.fhwa.saxton.carma.signal_plugin.ead;

import gov.dot.fhwa.saxton.carma.signal_plugin.ead.trajectorytree.ICostModel;
import gov.dot.fhwa.saxton.carma.signal_plugin.ead.trajectorytree.Node;
import gov.dot.fhwa.saxton.carma.signal_plugin.logger.ILogger;
import gov.dot.fhwa.saxton.carma.signal_plugin.logger.LoggerManager;

/**
 * Cost model for use in the trajectory tree solution. Computes a cost between neighboring nodes
 * based on travel time between those nodes.
 */
public class TimeCostModel implements ICostModel {

    private Node            goal_ = new Node(0, 0, 0);
    private Node            tolerances_ = new Node(0, 0, 0);
    private double          speedLimit_; // m/s
    private int             numCosts_ = 0;
    protected static ILogger log_ = LoggerManager.getLogger(TimeCostModel.class);


    /**
     * Builds the cost model object with injected calculation parameters.
     * @param speedLimit - roadway speed limit, m/s
     */
    public TimeCostModel(double speedLimit, double maxAccel) {
        speedLimit_ = speedLimit;
    }


    /**
     * Expresses the cost of moving from n1 to n2 in terms of travel time.
     * Checks for n2.distance >= n1.distance and n2.time > n1.time
     * This method assumes the nodes have been constructed within legal (i.e. speed limit) and physical (i.e.
     * vehicle capability limits) constraints.
     * @param n1 First node
     * @param n2 Second node
     * @return time, sec, or very large number if inputs are incorrect
     */
    @Override
    public double cost(Node n1, Node n2) {

        //input sanity checks
        if (n2.getTime() <= n1.getTime()  ||  n2.getDistance() < n1.getDistance()  ||
                n1.getSpeed() < 0  ||  n2.getSpeed() < 0) {
            log_.warn("PLAN", "Cost computation invoked with invalid nodes:");
            log_.warn("PLAN", "    n1: " + n1.toString());
            log_.warn("PLAN", "    n2: " + n2.toString());
            log_.warn("PLAN", "    " + numCosts_ + " costs have been evaluated since goal defined or previous error.");
            numCosts_ = 0;
            return Double.MAX_VALUE;
        }else {
            ++numCosts_;
        }

        return n2.getTimeAsDouble() - n1.getTimeAsDouble();
    }

    @Override
    public double heuristic(Node currentNode) {
        //no cost if we are already beyond the location of the goal (we are as close as our grid spacing will allow)
        if (currentNode.getDistance() >= goal_.getDistance()) {
            return 0.0;
        }

        //compute travel time between current location and goal at speed limit, ignoring signals
        double travelTime = (goal_.getDistanceAsDouble() - currentNode.getDistanceAsDouble()) / speedLimit_;
        //subtract a tiny amount of time to ensure this result is unreachably optimistic
        return travelTime - 0.1;
    }

    @Override
    public void setTolerances(Node tolerances) {
        tolerances_ = tolerances;
    }

    @Override
    public void setGoal(Node goal) {
        goal_ = goal;
        numCosts_ = 0;
    }

    /**
     * Note that speed comparison is not used, as we can assume that all speeds will be in the range of
     * [operating speed, speed limit], any of which will be suitable.
     */
    @Override
    public boolean isGoal(Node n) {
        boolean result = n.getDistance() >= goal_.getDistance() - tolerances_.getDistance()  &&
                         n.getTime()     <= goal_.getTime()     + tolerances_.getTime()      &&
                         n.getSpeed()    >= goal_.getSpeed()    - tolerances_.getSpeed()     &&
                         n.getSpeed()    <= goal_.getSpeed()    + tolerances_.getSpeed();

        if (result) {
            log_.debug("PLAN", "///// TimeCostModel has found a node that matches our goal.");
            log_.debug("PLAN", "    goal = " + goal_.toString());
            log_.debug("PLAN", "    node = " + n.toString());
        }

        return result;
    }

    @Override
    public boolean isUnusable(Node n) {
        //reject a node if its time is significantly larger than goal time or its distance is
        // significantly larger than goal distance
        long timeTol = tolerances_.getTime();
        long distTol = tolerances_.getDistance();

        return n.getTime() > goal_.getTime() + 3*timeTol  ||
                n.getDistance() > goal_.getDistance() + 3*distTol;
    }
}
