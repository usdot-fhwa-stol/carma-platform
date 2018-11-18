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
package gov.dot.fhwa.saxton.carma.signal_plugin.ead;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;

import gov.dot.fhwa.saxton.carma.signal_plugin.ead.trajectorytree.ICostModel;
import gov.dot.fhwa.saxton.carma.signal_plugin.ead.trajectorytree.Node;
import gov.dot.fhwa.saxton.carma.signal_plugin.logger.ILogger;
import gov.dot.fhwa.saxton.carma.signal_plugin.logger.LoggerManager;

/**
 * Cost model for use in the trajectory tree solution. Computes a cost between neighboring nodes
 * based on an estimation of fuel cost to travel between those nodes.
 *
 * This version uses the cost model presented in the UCR white paper, Glidepath II MOVES Brief, dated Dec 2017.
 * Some assumptions are made in this implementation.
 * It assumes a flat roadway (no grade) and uniform acceleration between the given nodes.
 * It also assumes acceleration <= -2.0 corresponds to deceleration
 * It also assumes that unknown operating modes have a cost equal to the highest cost present in the provided tables
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
    private final int EXPECTED_BASE_RATE_TABLE_LENGTH = 7; // Number of columns in the energy consumption table
    private final double SEC_PER_HR = 3600.0;
    private final double J_PER_KJ = 1000.0;
    private final double DEFAULT_PEAK_ENERGY_KJ; // The highest cost found in the energy consumption table

    private int numCosts = 0;
    protected static final ILogger log = LoggerManager.getLogger(FuelCostModel.class);
    protected static final double FUEL_COST_NORMALIZATION_DENOMINATOR = 425000.0;
    protected static final double TIME_COST_NORMALIZATION_DENOMINATOR = 2.0;
    protected static final double HEURISTICS_MULTIPLIER = 60.0;

    /**
     * Builds the cost model object with several injected calculation parameters.
     * These parameters should come from the EPA "MOVES2010 Highway Vehicle Population and Activity Data",
     * in accordance with the MOVES Brief document provided by UCR.
     * @param rollingTermA - Term corresponding to vehicle roll. Units: kW - s/m
     * @param rotatingTermB - Term corresponding to vehicle rotation. Units: kW - s^2/m^2
     * @param dragTermC - Term corresponding to vehicle drag. Units: kW - s^3/m^3
     * @param vehicleMassInTons - Source vehicle mass. Units: metric tons
     * @param fixedMassFactor - Fixed mass factor. Units: metric tons
     * @param baseRateTablePath - Path to the csv file used to store the energy consumption parameters based on vehicle state
     * @throws IOException - Exception thrown when the file specified by baseRateTablePath cannot be loaded properly
     */
    public MovesFuelCostModel(double rollingTermA, double rotatingTermB, double dragTermC, double vehicleMassInTons,
        double fixedMassFactor, String baseRateTablePath) throws IOException {
        //assign injected params
        this.rollingTermA = rollingTermA;
        this.rotatingTermB = rotatingTermB;
        this.dragTermC = dragTermC;
        this.vehicleMassInTons = vehicleMassInTons;
        this.fixedMassFactor = fixedMassFactor;
        this.baseRateTable = loadTable(baseRateTablePath); // Load the base rates table

        // Find the highest energy cost in the table and store it for use when values fall outside table scope
        double maxValue = 0; 
        for(Entry<Integer, List<Double>> entry: this.baseRateTable.entrySet()) {
            if (entry.getValue().get(BASE_RATE_ENERGY_COL) > maxValue) {
                maxValue = entry.getValue().get(BASE_RATE_ENERGY_COL);
            }
        }

        this.DEFAULT_PEAK_ENERGY_KJ = maxValue;
    }

    /**
     * Helper function to load a csv file containing the host vehicle energy consumption parameters based on vehicle state
     * 
     * @param baseRateTablePath - Path to the csv file used to store the energy consumption parameters based on vehicle state
     * @throws IOException - Exception thrown when the file specified by baseRateTablePath cannot be loaded properly
     * 
     * @return A mapping of operating mode id with energy consumption parameters 
     */
    private Map<Integer,List<Double>> loadTable(String baseRateTablePath) throws IOException {
        String line = "";
        String delimiter = ",";
        boolean firstLine = true;
        Map<Integer,List<Double>> map = new HashMap<>();

        try (BufferedReader br = new BufferedReader(new FileReader(baseRateTablePath))) {

            while ((line = br.readLine()) != null) {

                if (firstLine) {
                    firstLine = false;
                    continue;
                }
                String[] data = line.split(delimiter);

                if (data.length != EXPECTED_BASE_RATE_TABLE_LENGTH) {
                    throw new IOException("MOVES fuel cost model data file contained " + data.length + " columns but expected " + EXPECTED_BASE_RATE_TABLE_LENGTH);
                }

                Integer key = Integer.parseInt(data[0]);

                List<Double> values = new ArrayList<>(EXPECTED_BASE_RATE_TABLE_LENGTH - 1);

                for (int i = 1; i < data.length; i++) {
                    values.add(Double.parseDouble(data[i]));
                }

                map.put(key, values);
            }

        } catch (IOException e) {
            throw e;
        }
        return map;
    }


    /**
     * Expresses the cost of moving from n1 to n2 in terms of energy consumed.
     * Output cost is in units of KJ
     * Assumes n2.distance >= n1.distance and n2.time > n1.time
     * Calculation based on UCR MOVES Brief
     * 
     * @param n1 First node
     * @param n2 Second node
     * 
     * @return energy cost, J, or very large number if inputs are incorrect
     */
    @Override
    public double cost(Node n1, Node n2) {

        //input sanity checks
        if (n2.getTime() <= n1.getTime()  ||  n2.getDistance() < n1.getDistance()  ||
                n1.getSpeed() < 0  ||  n2.getSpeed() < 0) {
            log.debug("EAD", "Cost computation invoked with invalid nodes:");
            log.debug("EAD", "    n1: " + n1.toString());
            log.debug("EAD", "    n2: " + n2.toString());
            log.debug("EAD", "    " + numCosts + " costs have been evaluated since goal defined or previous error.");
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
        final double dt = n2.getTimeAsDouble() - n1.getTimeAsDouble(); // Change in time in seconds
        final double a = (v - n1.getSpeedAsDouble()) / dt; // a = dV / dt : We are using the acceleration to get from current speed to target speed
        final double theta = ROAD_GRADE; 

        // Calculate the VehicleSpecificPower (VSP)
        final double VSP = (A*v + B*v_sqr + C*v_sqr*v + M*v*(a + g * Math.sin(theta))) / f;

        // Determine Operating Mode from OpModeTable
        final int opMode = getModeConditional(VSP, v, a);

        // Return the highest cost which would still be in the table when our result is in the undefined region
        // Additionally log the occurrence
        if (opMode == -1) {
            log.debug("EAD", "Invalid operating mode found for MOVES cost calculation. Using highest known cost in table: " + DEFAULT_PEAK_ENERGY_KJ + " KJ");
            log.debug("EAD", "    Node 1: " + n1.toString());
            log.debug("EAD", "    Node 2: " + n2.toString());
            return J_PER_KJ * ((DEFAULT_PEAK_ENERGY_KJ / SEC_PER_HR) * dt);
        }

        // Get Parameters
        final List<Double> baseRateList = baseRateTable.get(opMode);

        double normalizedFuelCost = (J_PER_KJ * ((baseRateList.get(BASE_RATE_ENERGY_COL) / SEC_PER_HR) * dt)) / FUEL_COST_NORMALIZATION_DENOMINATOR;
        double normalizedTimeCost = (n2.getTime() - n1.getTime()) / TIME_COST_NORMALIZATION_DENOMINATOR;
        //System.out.println("normalizedFuelCost: " + normalizedFuelCost);
        // time cost: n2.getTime() - n1.getTime()
        return normalizedFuelCost + normalizedTimeCost; // Return the energy in J by converting KJ/hr to KJ/s and multiplying by dt and J/KJ
    }

    /**
     * Helper function for identifying the operating mode of the host vehicle based on vehicle specific power (VSP), velocity, and acceleration
     * 
     * This is a reconstruction of the conditional table lookup required by the UCR MOVES brief
     */
    private int getModeConditional(double VSP, double velocity, double acceleration) {
        // In the MOVES documentation this table is presented in mi/hr here it is converted to m/s to avoid conversions
        final double FIFTY_MPH_IN_MPS = 22.352;
        final double TWENTY_FIVE_MPH_IN_MPS = 11.176;
        final double ONE_MPH_IN_MPS = 0.44704;
        // NOTE: This acceleration check is a simplification of a <= -2 OR (a < -1.0 AND a_t-1 < -1.0 AND a_t-2 < -1.0)
        //       The simplification was made to avoid the need to get previous acceleration values into this function
        if (acceleration <= -1.0) {
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
        //infinite cost if we pass the location and the time of the goal
        if (currentNode.getDistance() > goal.getDistance() + tolerances.getDistance()) {
        	return Double.POSITIVE_INFINITY;
        }
        //smooth acceleration from current location to ending location & speed, ignoring the signal
        double normalizedHeuristics = (goal.getDistanceAsDouble() - currentNode.getDistanceAsDouble()) / goal.getDistanceAsDouble(); 
        //System.out.println("normalizedHeuristics: " + normalizedHeuristics);
        return normalizedHeuristics * HEURISTICS_MULTIPLIER;
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
            result = Math.abs(n.getDistance() - goal.getDistance()) <= tolerances.getDistance()  &&
                    Math.abs(n.getSpeed()    - goal.getSpeed())    <= tolerances.getSpeed();
        }else {
            result = n.getDistance() >= goal.getDistance()  &&
                     n.getSpeed()    >= goal.getSpeed();
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
        return n.getDistance() > (goal.getDistance() + tolerances.getDistance());
    }
}
