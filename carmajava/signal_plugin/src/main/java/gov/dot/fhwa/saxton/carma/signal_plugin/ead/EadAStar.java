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

import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.Constants;
import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.utils.GlidepathApplicationContext;
import gov.dot.fhwa.saxton.carma.signal_plugin.asd.IntersectionData;
import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.IGlidepathAppConfig;
import gov.dot.fhwa.saxton.carma.signal_plugin.ead.trajectorytree.*;
import gov.dot.fhwa.saxton.carma.signal_plugin.logger.ILogger;
import gov.dot.fhwa.saxton.carma.signal_plugin.logger.LoggerManager;

import java.util.List;

/**
 * Provides the logic to traverse multiple signalized intersections per white paper from UCR dated 8/18/17.
 */
public class EadAStar implements IEad {

    protected double                    maxAccel_;                  //maximum allowed acceleration, m/s^2
    protected List<Node>                currentPath_;               //path through the current intersection
    protected double                    ddt_ = 0.0;                 //distance downtrack of the current plan start, m
    protected double                    timeOnPath_ = 0.0;          //time since beginning the current path, sec
    protected boolean                   intListChanged_ = true;     //has the list of known intersections changed?
    protected List<IntersectionData>    intList_;                   //list of all known intersections
    protected int                       currentNodeIndex_ = 1;      //index to the next node downtrack in currentPath_
    protected double                    speedCmd_ = 0.0;            //speed command from the current node, m/s
    protected boolean                   firstCall_ = true;          //is this the first call to getTargetSpeed()?
    protected double				    timeStep_;			        //duration of one time step, sec
    protected long                      prevMethodStartTime_;       //time of the previous call to getTargetSpeed(), ms
    protected double				    stopBoxWidth_;			    //distance from one side of stop box to the other, meters
    protected double                    coarseTimeInc_;             //time increment for coarse planning, sec
    protected double                    coarseSpeedInc_;            //speed increment for coarse planning, m/s
    protected double                    fineTimeInc_;               //time increment for detailed planning, sec
    protected double                    fineSpeedInc_;              //speed increment for detailed planning, m/s
    protected double                    vehicleMass_;               //mass of the vehicle, kg
    protected double                    rollingRes_;                //coefficient of rolling resistance
    protected double                    dragCoef_;                  //drag coefficient
    protected double                    frontalArea_;               //vehicle frontal cross-section area, m^2
    protected double                    airDensity_;                //air density, kg/m^3
    protected double                    idlePower_;                 //brake power wasted at idle under no load, J/s
    protected boolean                   useIdleMin_;                //should we use idle power as min for all situations?
                                                                    // if not then it will only be used when speed = 0
    protected double                    maxDistanceError_;          //max allowable deviation from plan, m
    protected boolean                   replanNeeded_ = true;       //do we need to replan the trajectory?

    protected ICostModel                costModel_;                 //model of cost to travel between nodes in the tree
    protected INeighborCalculator       neighborCalc_;              //calculates neighboring nodes to build the tree
    protected ITreeSolver               solver_;                    //the tree solver
    protected static ILogger            log_ = LoggerManager.getLogger(EadAStar.class);



    public EadAStar() {

        IGlidepathAppConfig config = GlidepathApplicationContext.getInstance().getAppConfig();
        maxAccel_ = config.getDoubleDefaultValue("defaultAccel", 2.0);
        //params for tree node definitions
        coarseTimeInc_ = config.getDoubleDefaultValue("ead.coarse_time_inc", 5.0);
        coarseSpeedInc_ = config.getDoubleDefaultValue("ead.coarse_speed_inc", 3.0);
        fineTimeInc_ = config.getDoubleDefaultValue("ead.fine_time_inc", 2.0);
        fineSpeedInc_ = config.getDoubleDefaultValue("ead.fine_speed_inc", 1.0);
        //params for fuel cost model
        vehicleMass_ = config.getDoubleValue("ead.vehicleMass");
        rollingRes_ = config.getDoubleDefaultValue("ead.rollingResistanceOverride", 0.0);
        dragCoef_ = config.getDoubleValue("ead.dragCoefficient");
        frontalArea_ = config.getDoubleValue("ead.frontalArea");
        airDensity_ = config.getDoubleValue("ead.airDensity");
        idlePower_ = config.getDoubleValue("ead.idleCost");
        useIdleMin_ = config.getBooleanValue("ead.useIdleMin");

        //set the max distance error to be half the typical distance between nodes a nominal speed
        double speedLimit = (double)config.getMaximumSpeed(0.0) / Constants.MPS_TO_MPH;
        maxDistanceError_ = 0.5*speedLimit*fineTimeInc_;

        costModel_ = new FuelCostModel(vehicleMass_, rollingRes_, dragCoef_, frontalArea_, airDensity_,
                                        idlePower_, useIdleMin_);
        neighborCalc_ = new EadNeighborCalculator();
    }

    public List<Node> getCurrentPath() {
        return currentPath_;
    }

    @Override
    public void initialize(long timestep, ITreeSolver solver) {
        timeStep_ = (double)timestep / 1000.0;
        solver_ = solver;
    }

    @Override
    public void intersectionListHasChanged() {
        intListChanged_ = true;
    }

    @Override
    public void setStopBoxWidth(double width) {
        stopBoxWidth_ = width;
    }

    /**
     * NOTE that this method will only be called if vehicle is on a known approach lane to the nearest intersection.
     */
    @Override
    public double getTargetSpeed(double speed, double operSpeed, double accel,
                                 List<IntersectionData> intersections) throws Exception {

        if (intersections == null  ||  intersections.size() == 0) {
            String msg = "getTargetSpeed invoked with a empty intersection list.";
            log_.error("EAD", msg);
            throw new Exception(msg);
        }

        intList_ = intersections;
        long methodStartTime = System.currentTimeMillis();
        double dtsb = intersections.get(0).dtsb; //if we are close to the stop bar this will be a positive value
        double replanThreshold = 5.0*Math.max(speed, 2.0); //no need to replan if within 5 sec of stop bar

        if (firstCall_) {
            speedCmd_ = speed;
            firstCall_ = false;
        }


        ////////// BUILD & SOLVE THE DIJKSTRA TREE TO DEFINE THE BEST PATH


        //TODO: monitor the execution time of this block; if it is big we may need to put it in a separate thread

        //TODO: may be able to avoid replanning every time the intersection data is updated; should do so only when
        //      it is an advantage (e.g. no point if there is only 1 intersection and we are 3 sec from passing its bar)

        //if the intersection picture has changed since previous time step or we otherwise need to replan then
        if (replanNeeded_  ||  (intListChanged_  &&  dtsb > replanThreshold)) {
            Node goal;
            if (intListChanged_) {
                log_.info("EAD", "///// getTargetSpeed replanning begun due to changed intersection data.");
            }else {
                log_.info("EAD", "///// getTargetSpeed replanning begun due to other need.");
            }

            //define starting node and reset DDT since we are beginning execution of a new path
            // (starting the plan at commanded speed rather than actual speed will allow for smoother transitions
            // from one plan to another, since actual speed may be nowhere near commanded speed, but acceleration
            // is already trying to get us to the command quickly)
            if (Math.abs(speed - speedCmd_) > 2.0) {
                log_.warn("EAD", "New plan being generated when actual speed differs greatly from command."
                            + " prev command = " +  speedCmd_ + ", actual = " + speed);
            }
            Node startNode = new Node(0.0, 0.0, speedCmd_);
            currentNodeIndex_ = 0;
            ddt_ = 0.0;
            timeOnPath_ = 0.0;
            currentPath_ = null;

            //if there is more than one intersection in sight then
            if (intersections.size() > 1) {

                //perform coarse planning to determine the goal node downtrack of the first intersection [planCoarsePath]
                goal = planCoarsePath(operSpeed, startNode);

            //else (only one intersection is known)
            }else {

                //define a goal node downtrack of the intersection where we can recover operating speed)
                double dist = intersections.get(0).dtsb;
                if (dist < 0.0) {
                    dist = (double)intersections.get(0).roughDist / 100.0;
                    log_.debug("EAD", "    overriding with dist = " + dist);
                }
                dist += 0.5*operSpeed*operSpeed/maxAccel_; //worst case going from stop at red back to full speed
                goal = new Node(dist, 999.0, operSpeed); //large time allows heuristic calc to work
            }

            //build a detailed plan to get to the near-term goal node downtrack of first intersection [planDetailedPath]
            try {
                currentPath_ = planDetailedPath(startNode, goal);
            }catch (Exception e) {
                log_.warn("EAD", "getTargetSpeed trapped exception from planDetailedPath: " + e.toString());
            }

            if (currentPath_ == null  ||  currentPath_.size() == 0) {
                String msg = "getTargetSpeed produced an unusable detailed path.";
                log_.error("EAD", msg);
                throw new Exception(msg);
            }

            //clear the intersection changed indicator
            intListChanged_ = false;
            replanNeeded_ = false;

        //else (no change in intersections)
        }else {
            //compute the current distance downtrack from the beginning of the current plan
            double deltaTime = 0.001*((double)(methodStartTime - prevMethodStartTime_));
            timeOnPath_ += deltaTime;
            ddt_ += deltaTime*speed;
        }



        ////////// FOLLOW THE DEFINED PATH (TRAJECTORY)


        //determine which is the next node in the plan, based on time from start of plan
        Node currentNode = currentPath_.get(currentNodeIndex_);
        if (timeOnPath_ > currentNode.getTimeAsDouble()) {
            double error = ddt_ - currentNode.getDistanceAsDouble();
            log_.info("EAD", "Crossed detailed path node " + currentNodeIndex_ + " at time = "
                        + timeOnPath_ + " sec, dist = " + ddt_ + " m, dist error = " + error);

            //if we are sufficiently far off plan then indicate it's time to replan
            if (Math.abs(error) > maxDistanceError_) {
                replanNeeded_ = true;
                log_.info("EAD", "Performance has deviated too far from plan. Replan needed.");
            }

            //we should never reach the end of a plan, but check just in case
            if (currentNodeIndex_ < currentPath_.size() - 1) {
                //get the speed of the next downtrack node and use it as the new command
                ++currentNodeIndex_;
            }else {
                replanNeeded_ = true;
                String msg = "getTargetSpeed has fallen off the end of the path at node " + currentNodeIndex_;
                log_.warn("EAD", msg);
            }
        }

        //determine speed command by interpolating between the two nearest nodes - this avoids severe jerking
        // produced by the XGV controller when subjected to large step function of reduced speeds
        if (currentNodeIndex_ > 0) {
            Node prevNode = currentPath_.get(currentNodeIndex_ - 1);
            if (prevNode.getSpeed() == currentNode.getSpeed()) {
                speedCmd_ = currentNode.getSpeedAsDouble();
            }else {
                double prevSpeed = prevNode.getSpeedAsDouble();
                double deltaSpeed = currentNode.getSpeedAsDouble() - prevSpeed;
                double prevTime = prevNode.getTimeAsDouble();
                double deltaTime = currentNode.getTimeAsDouble() - prevTime;
                double f;
                try {
                    f = (timeOnPath_ - prevTime)/deltaTime;
                }catch (ArithmeticException e) { //should never happen
                    f = 1.0;
                    log_.warn("EAD", "Divide by zero when interpolating speed command.");
                }
                speedCmd_ = prevSpeed + f*deltaSpeed;
                log_.debug("EAD", "speed cmd interpolation: node = " + currentNodeIndex_ + ", prevSpeed = " + prevSpeed + ", deltaSpeed = "
                            + deltaSpeed + ", prevTime = " + prevTime + ", deltaTime = " + deltaTime + ", f = " + f);
            }
        }

        prevMethodStartTime_ = methodStartTime;
        long totalTime = System.currentTimeMillis() - methodStartTime;
        log_.debug("EAD", "getTargetSpeed completed in " + totalTime + " ms.");

        return speedCmd_;
    }

    ////////////////////
    // protected members
    ////////////////////

    /**
     * Defines a coarse-grained path through all the intersections to get a rough idea of the best long range trajectory.
     * Once this is complete, it will choose a node between the first (current) and second intersections that represents
     * the speed closest to our desired speed, and will return that as the goal node for detailed planning.
     * @return a node representing the best goal we can hope to reach after the first intersection
     * @apiNote success is defined when a path ends in a node that is within half of the increment between adjacent nodes
     * in the distance & speed dimensions (time dimension isn't compared).
     */
    protected Node planCoarsePath(double operSpeed, Node start) throws Exception {

        //we may be receiving spat signals from a farther intersection but haven't yet seen a MAP message from it,
        // so its rough distance is going to be a bogus, very large value; therefore, only look ahead for as many
        // intersections as we can see with maps defined
        int numInt = intList_.size(); //guaranteed to be >= 2
        int limit = numInt;
        for (int i = 0;  i < limit;  ++i) {
            if (intList_.get(i).map == null) {
                numInt = i;
                break;
            }
        }
        if (numInt == 0) {
            String msg = "planCoarsePath can't find any intersections with map data.";
            log_.warn("EAD", msg);
            throw new Exception(msg);
        }

        //define a goal node downtrack of the farthest known intersection (where we can recover operating speed)
        double dist = 0.01 * (double)intList_.get(numInt - 1).roughDist; //converting from cm to m
        double recoveryDist = 0.5*operSpeed*operSpeed/maxAccel_; //worst case going from stop at red back to full speed
        dist += recoveryDist;
        Node coarseGoal = new Node(dist, 999.0, operSpeed); //large time allows heuristic calculation to work
        log_.debug("EAD", "planCoarsePath entered with " + numInt + " useful intersections and a course goal of "
                    + coarseGoal.toString());

        //create a neighbor calculator for this tree using a coarse scale grid
        neighborCalc_.initialize(intList_, numInt, coarseTimeInc_, coarseSpeedInc_);

        //find the best path through this tree
        costModel_.setGoal(coarseGoal);
        costModel_.setTolerances(new Node(0.0, 0.0, 0.5*coarseSpeedInc_));
        List<Node> path = solver_.solve(start, costModel_, neighborCalc_);
        if (path == null  ||  path.size() == 0) {
            String msg = "planCoarsePath solver was unable to define a path.";
            log_.error("EAD", msg);
            throw new Exception(msg);
        }

        //find the node between first & second intersections whose speed is closest to the operating speed (due to
        // downtrack signal phases, it may not be practical to accelerate all the way to the operating speed in
        // between); use this as the goal for the detailed planning
        double adjustment = Math.max(recoveryDist - 0.5*coarseSpeedInc_*coarseTimeInc_, 0.5*recoveryDist);
        double startSearch = intList_.get(0).dtsb + adjustment;
        double endSearch = 0.01*(double)intList_.get(1).roughDist - adjustment;
        double maxSpeedFound = 0.0;
        Node tempFineGoal = null;
        for (Node n : path) {
            double curDist = n.getDistanceAsDouble();
            if (curDist > startSearch) {
                if (curDist > endSearch) {
                    log_.debug("EAD", "planCoarsePath found new intermediate goal with speed of " + maxSpeedFound);
                    break;
                }else if (n.getSpeedAsDouble() > maxSpeedFound) {
                    tempFineGoal = n;
                    maxSpeedFound = n.getSpeedAsDouble();
                }
            }
        }

        //ensure the time for the new goal node is sufficiently large
        Node fineGoal = new Node(tempFineGoal.getDistance(), 2*tempFineGoal.getTime(), tempFineGoal.getSpeed());

        //evaluate the chosen path [summarizeCoarsePath]
        summarizeCoarsePath(path, coarseGoal, fineGoal);

        return fineGoal;
    }


    /**
     * Finds the best fine-grained path from current location to the far side of the nearest intersection.
     * @param goal - the node beyond the nearest intersection that we are trying to reach
     * @return - the best path to the goal node
     * @apiNote neighbor nodes are located according to the increments in each dimension specified by the
     * config file parameters fineTimeInc, fineDistInc and fineSpeedInc.
     */
    protected List<Node> planDetailedPath(Node start, Node goal) throws Exception {
        log_.debug("EAD", "Entering planDetailedPath with start = " + start.toString() + ", goal = " + goal.toString());

        //create a neighbor calculator for this tree using a detailed grid
        neighborCalc_.initialize(intList_, 1, fineTimeInc_, fineSpeedInc_);

        //find the best path through this tree [use AStarSolver]
        costModel_.setGoal(goal);
        costModel_.setTolerances(new Node(0.0, 0.0, 0.5*fineSpeedInc_));
        List<Node> path = solver_.solve(start, costModel_, neighborCalc_);
        if (path == null  ||  path.size() == 0) {
            String msg = "///// planDetailedPath solver was unable to define a path.";
            log_.error("EAD", msg);
            throw new Exception(msg);
        }
        log_.debug("EAD", "planDetailedPath completed a path with " + path.size() + " nodes.");

        //evaluate the chosen path [summarizeDetailedPath]
        summarizeDetailedPath(path, goal);

        return path;
    }


    /**
     * Logs pertinent info about the coarse solution for human consumption.
     */
    protected void summarizeCoarsePath(List<Node> path, Node coarseGoal, Node fineGoal) {

        log_.debug("EAD", "///// Coarse plan covered " + intList_.size() + " intersections at distances of:");
        log_.debugf("EAD", "    %.0f m", intList_.get(0).dtsb);
        for (int i = 1;  i < intList_.size();  ++i) {
            log_.debugf("EAD", "    %.0f m", 0.01 * (double)intList_.get(i).roughDist);
        }
        log_.debug("EAD", "Coarse path plan:");
        for (Node n : path) {
            log_.debug("EAD", "    " + n.toString());
        }
        log_.debug("EAD", "Coarse path attempted to reach goal: " + coarseGoal.toString());
        log_.debug("EAD", "Coarse path yielded goal for detailed planning of: " + fineGoal.toString());
    }


    /**
     * Logs pertinent info about the detailed solution through the current intersection for human consumption.
     */
    protected void summarizeDetailedPath(List<Node> path, Node goal) {

        log_.debug("EAD", "///// Detailed path plan:");
        for (Node n : path) {
            log_.debug("EAD", "    " + n.toString());
        }
        log_.debug("EAD", "Detailed path attempted to reach goal: " + goal.toString());
    }
}
