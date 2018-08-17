package gov.dot.fhwa.saxton.carma.signal_plugin.ead;

import java.util.List;
import java.util.Map;

import gov.dot.fhwa.saxton.carma.signal_plugin.ead.trajectorytree.ICostModel;
import gov.dot.fhwa.saxton.carma.signal_plugin.ead.trajectorytree.Node;
import gov.dot.fhwa.saxton.carma.signal_plugin.logger.ILogger;
import gov.dot.fhwa.saxton.carma.signal_plugin.logger.LoggerManager;

/**
 * Cost model for use in the trajectory tree solution. Computes a cost between neighboring nodes
 * based on an estimation of fuel cost to travel between those nodes.
 *
 * This version uses equation (3) of the UCR white paper, Glidepath II Algorithm, dated Aug 2017.
 * It assumes a flat roadway (no grade) and uniform acceleration between the given nodes.
 * It uses simple average speed between the beginning and ending nodes rather than integrating.
 * It also assumes some fixed, positive cost at zero speed to represent fuel consumption at idle.
 */
public class MovesFuelCostModel implements ICostModel {

    // Goal evaluation variables
    private Node            goal = new Node(0, 0, 0);
    private Node            tolerances = null;

    // Cost calculation variables
    private final double rollingTermA;
    private final double rotatingTermB;
    private final double dragTermC;
    private final double vehicleMassInTons;
    private final double fixedMassFactor;
    private final Map<Integer,List<Double>> baseRateTable;
    private final int BASE_RATE_ENERGY_COL = 4;
    private final double ROAD_GRADE = 0.0; // By default we assume the road is flat. 0.0 in rad

    private int numCosts = 0;
    protected static ILogger log = LoggerManager.getLogger(FuelCostModel.class);


    /**
     * Builds the cost model object with several injected calculation parameters.
     * TODO
     * @param mass - vehicle mass, kg
     * @param rollRes - rolling resistance coefficient override; if this is 0 then internal calc will be used
     * @param dragCoef - vehicle aerodynamic drag coefficient
     * @param area - vehicle frontal cross-sectional area, m^2
     * @param airDensity - air density, kg/m^3
     * @param idleCost - rate of energy consumption at zero speed due to engine idling, J/s
     * @param useIdleMin - if true the the idleCost will be the lower limit returned for all situations;
     *                   if false then idleCost will only be applied if speed is zero
     */
    public MovesFuelCostModel(double rollingTermA, double rotatingTermB, double dragTermC, double vehicleMassInTons,
        double fixedMassFactor, String baseRateTablePath) {
        //assign injected params for mass, rolling resistance, drag coefficient, frontal area, air density, idle cost
        this.rollingTermA = rollingTermA;
        this.rotatingTermB = rotatingTermB;
        this.dragTermC = dragTermC;
        this.vehicleMassInTons = vehicleMassInTons;
        this.fixedMassFactor = fixedMassFactor;
        this.baseRateTable = baseRateTable;
    }


    /**
     * Expresses the cost of moving from n1 to n2 in terms of energy consumed.
     * Assumes n2.distance >= n1.distance and n2.time > n1.time
     * @param n1 First node
     * @param n2 Second node
     * @return energy cost, J, or very large number if inputs are incorrect
     */
    @Override
    public double cost(Node n1, Node n2) {

        //input sanity checks
        if (n2.getTime() <= n1.getTime()  ||  n2.getDistance() < n1.getDistance()  ||
                n1.getSpeed() < 0  ||  n2.getSpeed() < 0) {
            log.warn("EAD", "Cost computation invoked with invalid nodes:");
            log.warn("EAD", "    n1: " + n1.toString());
            log.warn("EAD", "    n2: " + n2.toString());
            log.warn("EAD", "    " + numCosts + " costs have been evaluated since goal defined or previous error.");
            numCosts = 0;
            return Double.MAX_VALUE;
        }else {
            ++numCosts;
        }

        // Get VSP
        final double A = rollingTermA;
        final double B = rotatingTermB;
        final double C = dragTermC;
        final double M = vehicleMassInTons;
        final double f = fixedMassFactor;
        final double g = 9.8; // Acceleration due to gravity = 9.8 m/s^2
        final double v = n2.getSpeedAsDouble(); // We are using the target node's velocity
        final double v_sqr = v*v;
        final double a = (v - n1.getSpeedAsDouble()) / (n2.getTimeAsDouble() - n1.getTimeAsDouble()); // a = dV / t : We are using the acceleration to get from current speed to target speed
        final double theta = ROAD_GRADE; 

        // Calculate the VehicleSpecificPower (VSP)
        final double VSP = (A*v + B*v_sqr + C*v_sqr + M*v*(a + g * Math.sin(theta))) / f;

        // Determine Operating Mode from OpModeTable
        final int opMode = getModeConditional(VSP, v, a);

        // Get Parameters
        final List<Double> baseRateList = baseRateTable.get(opMode);

        return baseRateList.get(BASE_RATE_ENERGY_COL); // Return the energy in KJ/hr
    }

    private int getModeConditional(double VSP, double velocity, double acceleration) {
        // In the MOVES documentation this table is presented in mi/hr here it is converted to m/s to avoid conversions
        final double FIFTY_MPH_IN_MPS = 22.352;
        final double TWENTY_FIVE_MPH_IN_MPS = 11.176;
        final double ONE_MPH_IN_MPS = 0.44704;
        // NOTE: This acceleration check is a simplification of a <= -2 OR (a < -1.0 AND a_t-1 < -1.0 AND a_t-2 < -1.0)
        //       The simplification was made to avoid the need to get previous acceleration values into this function
        if (acceleration <= -2.0) {
            return 0;
        }
        if (velocity >= FIFTY_MPH_IN_MPS) {
          if (VSP < 6.0) {
            return 33;
          } else if (6<= VSP && VSP < 12) {
            return 35;
          } else if (12<= VSP && VSP < 18) {
            return 37;
          } else if (18 <= VSP && VSP < 24) {
            return 38;
          } else if (24 <= VSP && VSP < 30) {
            return 39;
          } else {
            return 40;
          }
    
        } else if (-ONE_MPH_IN_MPS <= velocity && velocity < ONE_MPH_IN_MPS) {
          return 1;
        } else if (TWENTY_FIVE_MPH_IN_MPS <= velocity && velocity < FIFTY_MPH_IN_MPS) {
          if (VSP < 0) {
            return 21;
          } else if (0 <= VSP && VSP < 3) {
            return 22;
          } else if (3 <= VSP && VSP < 6) {
            return 23;
          } else if (6 <= VSP && VSP < 9) {
            return 24;
          } else if (9 <= VSP && VSP < 12) {
            return 25;
          } else if (12 <= VSP && VSP < 18) {
            return 27;
          } else if (18 <= VSP && VSP < 24) {
            return 28;
          }  else if (24 <= VSP && VSP < 30) {
            return 29;
          } else {
            return 30;
          }
        } else if (0.0 <= velocity && velocity < TWENTY_FIVE_MPH_IN_MPS) {
          if (VSP < 0) {
            return 11;
          } else if (0 <= VSP && VSP < 3) {
            return 12;
          } else if (3 <= VSP && VSP < 6) {
            return 13;
          } else if (6 <= VSP && VSP < 9) {
            return 14;
          } else if (9 <= VSP && VSP < 12) {
            return 15;
          } else {
            return 16;
          }
        } else {
          return -1;
        }
    }

    @Override
    public double heuristic(Node currentNode) {
        //no cost if we are already beyond the location of the goal (we are as close as our grid spacing will allow)
        if (currentNode.getDistance() >= goal.getDistance()) {
            return 0.0;
        }
        //smooth acceleration from current location to ending location & speed, ignoring the signal
        return cost(currentNode, goal);
    }

    @Override
    public void setTolerances(Node tolerances) {
        this.tolerances = tolerances;
    }

    @Override
    public void setGoal(Node goal) {
        this.goal = goal;
        this.numCosts = 0;
    }

    /**
     * Note that time comparisons are currently disabled because we don't have a good way to anticipate how much
     * time might be spent waiting at a red light, or not.
     */
    @Override
    public boolean isGoal(Node n) {
        boolean result;

        //if tolerances have been specified then use them
        if (tolerances != null) {
            result = n.getDistance() >= goal.getDistance()  &&
                   //Math.abs(n.getTime()     - goal_.getTime())     <= tolerances_.getTime()     &&
                     Math.abs(n.getSpeed()    - goal.getSpeed())    <= tolerances.getSpeed();
        }else {
            result = n.getDistance() == goal.getDistance()  &&
                   //n.getTime()     == goal_.getTime()      &&
                     n.getSpeed()    == goal.getSpeed();
        }

        if (result) {
            log.debug("EAD", "///// isGoal has found a node that matches our goal.");
            log.debug("EAD", "    goal = " + goal.toString());
            log.debug("EAD", "    node = " + n.toString());
            log.debug("EAD", "    tol  = " + tolerances.toString());
        }

        return result;
    }

    @Override
    public boolean isUnusable(Node n) {
        //if this node is more than one time increment beyond the goal distance and not near the goal speed
        // then there is no point in considering it or its children
        long excessDistance = n.getSpeed() * n.getTime();
        if (n.getDistance() > goal.getDistance() + excessDistance) {
            if (tolerances != null) {
                if (Math.abs(n.getSpeed() - goal.getSpeed()) > tolerances.getSpeed()) {
                    return true;
                }
            }else {
                if (n.getSpeed() != goal.getSpeed()) {
                    return true;
                }
            }
        }

        //if the node's time is significantly larger than the goal node's time, reject it
        if (n.getTimeAsDouble() > goal.getTimeAsDouble() + 5.0) {
            return true;
        }

        return false;
    }
}
