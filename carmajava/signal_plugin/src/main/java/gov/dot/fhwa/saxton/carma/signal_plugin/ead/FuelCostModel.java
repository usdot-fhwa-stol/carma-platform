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

import gov.dot.fhwa.saxton.carma.signal_plugin.ead.trajectorytree.ICostModel;
import gov.dot.fhwa.saxton.carma.signal_plugin.ead.trajectorytree.Node;
import gov.dot.fhwa.saxton.carma.signal_plugin.logger.ILogger;
import gov.dot.fhwa.saxton.carma.signal_plugin.logger.LoggerManager;

/**
 * 
 * TODO: This model has been deprecated for MOVES and has therefore not been updated for new algorithm changes as of 12/5/18
 * 
 * Cost model for use in the trajectory tree solution. Computes a cost between neighboring nodes
 * based on an estimation of fuel cost to travel between those nodes.
 *
 * This version uses equation (3) of the UCR white paper, Glidepath II Algorithm, dated Aug 2017.
 * It assumes a flat roadway (no grade) and uniform acceleration between the given nodes.
 * It uses simple average speed between the beginning and ending nodes rather than integrating.
 * It also assumes some fixed, positive cost at zero speed to represent fuel consumption at idle.
 */
public class FuelCostModel implements ICostModel {

    private Node            goal_ = new Node(0, 0, 0);
    private Node            tolerances_ = null;
    private double          mass_;
    private double          rollRes_;
    private double          dragCoef_;
    private double          area_;
    private double          airDensity_;
    private double          idleCost_;
    private boolean         useIdleMin_;
    private int             numCosts_ = 0;
    protected static final ILogger log_ = LoggerManager.getLogger(FuelCostModel.class);


    /**
     * Builds the cost model object with several injected calculation parameters.
     * @param mass - vehicle mass, kg
     * @param rollRes - rolling resistance coefficient override; if this is 0 then internal calc will be used
     * @param dragCoef - vehicle aerodynamic drag coefficient
     * @param area - vehicle frontal cross-sectional area, m^2
     * @param airDensity - air density, kg/m^3
     * @param idleCost - rate of energy consumption at zero speed due to engine idling, J/s
     * @param useIdleMin - if true the the idleCost will be the lower limit returned for all situations;
     *                   if false then idleCost will only be applied if speed is zero
     */
    public FuelCostModel(double mass, double rollRes, double dragCoef, double area, double airDensity,
                         double idleCost, boolean useIdleMin) {
        //assign injected params for mass, rolling resistance, drag coefficient, frontal area, air density, idle cost
        mass_ = mass;
        rollRes_ = rollRes;
        dragCoef_ = dragCoef;
        area_ = area;
        airDensity_ = airDensity;
        idleCost_ = idleCost;
        useIdleMin_ = useIdleMin;
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
            log_.debug("EAD", "Cost computation invoked with invalid nodes:");
            log_.debug("EAD", "    n1: " + n1.toString());
            log_.debug("EAD", "    n2: " + n2.toString());
            log_.debug("EAD", "    " + numCosts_ + " costs have been evaluated since goal defined or previous error.");
            numCosts_ = 0;
            return Double.MAX_VALUE;
        }else {
            ++numCosts_;
        }

        final double TIRE_PRESSURE = 2.0; //bar (2.0 bar ~= 29 psi)
        double avgSpeed = 0.5*(n1.getSpeedAsDouble() + n2.getSpeedAsDouble()); // m/s
        double length = n2.getDistanceAsDouble() - n1.getDistanceAsDouble();
        double duration = n2.getTimeAsDouble() - n1.getTimeAsDouble();
        //log_.debug("EAD", "cost entered w/avgSpeed = " + avgSpeed + ", length = " + length +
        //          ", duration = " + duration);

        //compute the rolling resistance as a function of speed using the coefficient override if supplied
        // per https://www.engineeringtoolbox.com/rolling-friction-resistance-d_1303.html
        double coefRollingRes;
        if (rollRes_ > 0.0) {
            coefRollingRes = rollRes_;
        }else {
            double kmhr = avgSpeed * 3.6; // km/hr
            //calculation from https://www.engineeringtoolbox.com/rolling-friction-resistance-d_1303.html
            coefRollingRes = 0.005 + (1.0 / TIRE_PRESSURE) * (0.01 + 0.0095 * (kmhr * kmhr / 10000.0));
        }
        double forceRollingRes = 9.81*mass_*coefRollingRes;

        //compute the air drag
        double forceDrag = 0.5*dragCoef_*airDensity_*area_*avgSpeed*avgSpeed;

        //compute grade effect, the force required to climb a hill - assume a flat track for current version
        double forceGrade = 0.0;

        //compute inertia force (limit to a non-negative value, as we won't consume fuel to slow the vehicle)
        double accel = (n2.getSpeedAsDouble() - n1.getSpeedAsDouble()) / duration;
        double forceInertia = 0.0;
        if (accel > 0.0) {
            forceInertia = mass_ * accel;
        }
        //log_.debug("EAD", "cost: coefRollingRes = " + coefRollingRes +
        //                ", forceRollingRes = " + forceRollingRes +
        //                ", forceDrag = " + forceDrag +
        //                ", accel = " + accel + ", forceInertia = " + forceInertia);

        //compute total work expended over the distance between these nodes
        double work = (forceRollingRes + forceDrag + forceGrade + forceInertia) * length; // Joules

        //compute energy consumption if the whole thing was done at idle
        double idleEnergy = idleCost_* duration;
        //log_.debug("EAD", "cost: preliminary work = " + work + ", idleEnergy = " + idleEnergy);

        //if we are using idle consumption as a minimum then
        if (useIdleMin_) {
            //limit the energy consumption to be at least as large as that used at no-load idle
            work = Math.max(work, idleEnergy);
        //else if speed is zero then
        }else if(avgSpeed < 0.5) {
            //compute based on idle consumption rate
            work = idleEnergy;
        }
        //log_.debug("EAD", "cost: returning work = " + work);

        return work;
    }

    @Override
    public double heuristic(Node currentNode) {
        //infinite cost if we pass the location and the time of the goal
        if (currentNode.getDistance() > (goal_.getDistance() + tolerances_.getDistance())) {
            return Double.POSITIVE_INFINITY;
        }
        //smooth acceleration from current location to ending location & speed, ignoring the signal
        return 0;
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

    @Override
    public boolean isGoal(Node n) {
        boolean result;

        //if tolerances have been specified then use them
        if (tolerances_ != null) {
            result = Math.abs(n.getDistance() - goal_.getDistance()) <= tolerances_.getDistance()  &&
                     Math.abs(n.getSpeed()    - goal_.getSpeed())    <= tolerances_.getSpeed();
        } else {
            result = n.getDistance() >= goal_.getDistance()  &&
                     n.getSpeed()    >= goal_.getSpeed();
        }

        if (result) {
            log_.debug("EAD", "///// isGoal has found a node that matches our goal.");
            log_.debug("EAD", "    goal = " + goal_.toString());
            log_.debug("EAD", "    node = " + n.toString());
        }

        return result;
    }

    @Override
    public boolean isUnusable(Node n) {
        return n.getDistance() > (goal_.getDistance() + tolerances_.getDistance());
    }
}