/*
 * Copyright (C) 2018-2019 LEIDOS.
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

import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.SignalPhase;
import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.utils.GlidepathApplicationContext;
import gov.dot.fhwa.saxton.carma.signal_plugin.asd.IntersectionData;
import gov.dot.fhwa.saxton.carma.signal_plugin.ead.INodeCollisionChecker;
import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.IGlidepathAppConfig;
import gov.dot.fhwa.saxton.carma.signal_plugin.logger.ILogger;
import gov.dot.fhwa.saxton.carma.signal_plugin.logger.LoggerManager;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

import static gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.Constants.MPS_TO_MPH;

/**
 * Calculates the viable neighbors for a node in the fine grid solution tree for the EAD model.
 */
public class FinePathNeighbors extends NeighborBase {

    protected double                        acceptableStopDist_; //max dist before bar it's acceptable to stop, m
    protected double                        debugThreshold_;//node distance beyond which we will turn on debug logging
    protected double                        responseLag_; //vehicle dynamic response lag, sec
    protected static final ILogger                log_ = LoggerManager.getLogger(FinePathNeighbors.class);
    protected static final double                 FLOATING_POINT_EPSILON = 0.1;

    protected static int prevAccelDir = -2;
    public FinePathNeighbors() {
        history_ = new ArrayList<>();

        //get config parameters
        IGlidepathAppConfig config = GlidepathApplicationContext.getInstance().getAppConfig();
        maxAccel_ = config.getDoubleDefaultValue("defaultAccel", 2.0);
        speedLimit_ = config.getMaximumSpeed(0.0) / MPS_TO_MPH;
        crawlingSpeed_ = config.getDoubleDefaultValue("crawlingSpeed", 5.0) / MPS_TO_MPH;
        acceptableStopDist_ = config.getDoubleDefaultValue("ead.acceptableStopDistance", 6.0);
        timeBuffer_ = config.getDoubleDefaultValue("ead.timebuffer", 4.0);
        debugThreshold_ = config.getDoubleDefaultValue("ead.debugThreshold", -1.0);
        responseLag_ = config.getDoubleDefaultValue("ead.response.lag", 1.9);
    }


    @Override
    public void initialize(List<IntersectionData> intersections, int numIntersections, double timeIncrement,
                    double speedIncrement, INodeCollisionChecker collisionChecker, double planningStartTime, double planningStartDowntrack) {

        log_.info("EAD", "initialize called with timeInc = " + timeIncrement + ", speedInc = " + speedIncrement);
        // Set the acceptable stop distance to at least half the distance increment
        acceptableStopDist_ = Math.max(1.1 * 2.0 * timeIncrement * speedIncrement, acceptableStopDist_); 
        log_.info("EAD", "Using acceptable stop distance of: " + acceptableStopDist_);
        super.initialize(intersections, numIntersections, timeIncrement, speedIncrement, collisionChecker, planningStartTime, planningStartDowntrack);
    }

    @Override
    public List<Node> neighbors(Node node) {
        List<Node> neighbors = new ArrayList<>();
        int c = currentIntersectionIndex(node.getDistanceAsDouble());
        if (c >= 0 ) {
            SignalState sigState = phaseAtTime(currentIntersectionIndex(node.getDistanceAsDouble()), node.getTimeAsDouble());
            //System.out.println("Generating neighbors of node " + node.toString() + " Phase: " + sigState.phase + " timeRemaining: " + sigState.timeRemaining);
        } else {
            //System.out.println("Generating neighbors of node " + node.toString() + " Past last intersection:");
        }

        double curTime = node.getTimeAsDouble();
        double curDist = node.getDistanceAsDouble();
        double curSpeed = node.getSpeedAsDouble();
        
        //if this node will inevitably result in signal violation, return no neighbors
        double timeToStop = curSpeed / maxAccel_;
        double distToStop = timeToStop * curSpeed * 0.5;
        
        // TODO I am not convinved this check is any guarentee of invalid behavior
        // if(signalViolation(node, new Node(curDist + distToStop, timeToStop + curTime, 0.0))) {
        //     //System.out.println("this node has no neighbor because of signal violation: " + node);
        //     //System.out.println();
        //     return neighbors;
        // }

        double variableTimeInc = Math.max(timeInc_, responseLag_);
        //System.out.println("TimeInc: " + variableTimeInc);
        double newTime = curTime + variableTimeInc;

        List<Double> speeds = getViableSpeeds(node, variableTimeInc);
        
        //loop on the reachable speed increments
        for(double v : speeds) {
            double deltaD = variableTimeInc * (curSpeed + v) * 0.5;
            double newDist = curDist + deltaD;
            Node potentialNeighbor = new Node(newDist, newTime, v);
            //System.out.println("Potential Neighbor: " + potentialNeighbor.toString());
            if(!signalViolation(node, potentialNeighbor)) {

                neighbors.add(potentialNeighbor);
            } else {
                //System.out.println("Signal Violation N1: " + node + " n2: " + potentialNeighbor); 
                log_.debug("PLAN", "Remove candidate speed due to signal violation: " + v);
            }
        }
        log_.debug("PLAN", "Generating neighbors of size " + neighbors.size());
        
        //System.out.println();
        List<Node> neightborsWithoutConflicts = neighbors.stream().filter(n -> !hasConflict(node, n)).collect(Collectors.toList());

        // No non collision solution exists so stop the vehicle
        if (neighbors.size() != 0 && neightborsWithoutConflicts.size() == 0) {
            // TODO this node needs to be given more thought
            neighbors.add(new Node(curDist, newTime, 0.0));
        } else if (neighbors.size() == 0 && curSpeed < FLOATING_POINT_EPSILON) {
            //System.out.println("Stopped and no neighbors must be at stop bar so adding 0 speed node");
            //System.out.println("Potential Neighbor: " + new Node(curDist, newTime, 0.0).toString());
            neighbors.add(new Node(curDist, newTime, 0.0));
        }

        return neightborsWithoutConflicts;
    }
    
    ////////////////////

    protected List<Double> getViableSpeeds(Node node, double variableTimeInc) {
        double curTime = node.getTimeAsDouble();
        double curDist = node.getDistanceAsDouble();
        double curSpeed = node.getSpeedAsDouble();

        List<Double> speeds = new ArrayList<>();

        //get candidate upper & lower speeds based on acceleration limit, time increment and speed limit
        // we will create nodes at regular increments between these
        double minSpeed = Math.max(curSpeed - maxAccel_*variableTimeInc, 0.0);
        double maxSpeed = Math.min(curSpeed + maxAccel_*variableTimeInc, speedLimit_);
        final double roundedCrawlingSpeed = crawlingSpeed_;
        //////System.out.println("Initial minSpeed = " + minSpeed + ", maxSpeed = " + maxSpeed);

        int currentInt = currentIntersectionIndex(curDist);
        double dtsb = distToIntersection(currentInt, curDist);
        // If the current node is a zero speed node within the stop distance then a stop has occurred, any future node should be 0.0 unless the phase will be green
        if (dtsb <= acceptableStopDist_ && currentInt >= 0) {
            boolean nextTimeIsGreen = phaseAtTime(currentInt, curTime + variableTimeInc - (timeBuffer_)).phase.equals(SignalPhase.GREEN);
            if (curSpeed < FLOATING_POINT_EPSILON && !nextTimeIsGreen) {
                speeds.add(0.0);
                return speeds;
            }
        }
        ////System.out.println("Speeds1: " + speeds);

        //we only allow small speed around acceptableStopDist_ and will be capped to 0.0
        if(dtsb <= acceptableStopDist_ && minSpeed < FLOATING_POINT_EPSILON && curSpeed > FLOATING_POINT_EPSILON) {
            speeds.add(0.0);
        } else {
            if (FinePathNeighbors.prevAccelDir == -1) {
                maxSpeed = curSpeed;
            } else if (FinePathNeighbors.prevAccelDir == 1) {
                minSpeed = curSpeed;
            } else {
    
            }
            speeds.add(Math.max(minSpeed, roundedCrawlingSpeed));
        }
        if (FinePathNeighbors.prevAccelDir == -1) {
            maxSpeed = curSpeed;
        } else if (FinePathNeighbors.prevAccelDir == 1) {
            minSpeed = curSpeed;
        } else {

        }

        ////System.out.println("Speeds2: " + speeds);

        double newSpeed = curSpeed;
        //add current speed if it is larger than crawling speed
        if(curSpeed > roundedCrawlingSpeed) {
            speeds.add(curSpeed);
        }

        ////System.out.println("Speeds3: " + speeds);
        //decrement from the current speed minus speedInc until the minSpeed
        newSpeed = curSpeed - speedInc_;
        while(newSpeed > Node.roundToSpeedUnits(minSpeed)) {
            if (newSpeed > roundedCrawlingSpeed) {
                speeds.add(newSpeed);
            }
            newSpeed -= speedInc_;
        }

        ////System.out.println("Speeds4: " + speeds);

        //increment from the current speed plus speedInc until the maxSpeed limit
        newSpeed = curSpeed + speedInc_;
        while(newSpeed < Node.roundToSpeedUnits(maxSpeed)) {
            if (newSpeed > roundedCrawlingSpeed) {
                speeds.add(newSpeed);
            }
            newSpeed += speedInc_; 
        }
        speeds.add(maxSpeed);

        ////System.out.println("Speeds5: " + speeds);
        return speeds;
    }

    /**
     * Determines if the vehicle would violate signal laws when travelling from the start to final conditions specified
     * @param startDist - starting distance downtrack of plan start, m
     * @param endDist - final distance downtrack of plan start, m
     * @param startTime - starting time since beginning of plan, sec
     * @param endTime - final time since start of plan, sec
     * @return true if a violation would occur (runs a red light) while moving from start to end
     */
    protected boolean signalViolation(Node n1, Node n2) {
        //////System.out.println("Entering signalViolation: startDist = " + startDist + ", endDist = "
       //            + endDist + ", startTime = " + startTime + ", endTime = " + endTime);
        final double startDist = n1.getDistanceAsDouble();
        final double startTime = n1.getTimeAsDouble(); 
        final double startSpeed = n1.getSpeedAsDouble();

        final double endDist = n2.getDistanceAsDouble();
        final double endTime = n2.getTimeAsDouble();
        final double endSpeed = n2.getSpeedAsDouble();
        
        //determine which intersection we are approaching [currentIntersectionIndex]
        int currentInt = currentIntersectionIndex(startDist);
        //////System.out.println("IntersectionIdex: " + currentInt);
        if (currentInt == -1) {
           //System.out.println("Violates: false currentInt: " + currentInt);
            return false;
        }

        //if the travel from start to end is not going to cross the stop bar then
        double distToInt = distToIntersection(currentInt, endDist);
        // Special case check for when we are stopping slightly passed the stop bar
        if (Node.roundToSpeedUnits(endSpeed) == 0 && distToInt == 0.0) {
            return false;
        }

        if (distToInt > 0.0) { // Check if we are within one distance unit of the stop bar or before it
           //System.out.println("Violates: false distToInt: " + distToInt + " startDist: " + startDist + " endDist: " + endDist + " startTime: " + startTime + ", endTime: " + endTime);
            return false;
        }

        double dtsb = distToIntersection(currentInt, 0.0) - startDist;
        if (dtsb < 0) {
            //System.out.println("Violates: false DTSB: " + dtsb);
            return false;
        }

        final double acceleration = (endSpeed - startSpeed) / (endTime - startTime);

        // Compute velocity at stop bar
        double crossingTime = startTime;
        if (Math.abs(acceleration) < FLOATING_POINT_EPSILON) {
            crossingTime += dtsb / startSpeed;
        } else {
            double vf = Math.sqrt(startSpeed * startSpeed + 2.0 * dtsb * acceleration);
            crossingTime += ((vf - startSpeed) /  acceleration);
        }
        //log_.debug("EAD", "Stop bar will be crossed: interpFactor = " + interpFactor
        //            + ", crossingTime = " + crossingTime);

        
        //to account for uncertainties in the vehicle's dynamic response to speed command changes, we need
        // a little wiggle room, so don't want to cross the bar just as signal is changing color; we want
        // to avoid red at crossing time +/- the specified buffer (use a smaller time buffer to make fine plan easier)
        SignalPhase earlyPhase = phaseAtTime(currentInt, crossingTime - (timeBuffer_)).phase;
        SignalPhase latePhase = phaseAtTime(currentInt, crossingTime + (timeBuffer_)).phase;
        boolean redIfEarly = earlyPhase == SignalPhase.RED || earlyPhase == SignalPhase.YELLOW; 
        boolean redIfLate  = latePhase == SignalPhase.RED || latePhase == SignalPhase.YELLOW;

        //System.out.println("Violates: " + (redIfEarly || redIfLate) + " redIfEarly: " + redIfEarly + " redIfLate: " + redIfLate);
        return redIfEarly || redIfLate;
    }


    private void logDebug(double distance, String msg) {
        if (debugThreshold_ >= 0.0  &&  distance >= debugThreshold_) {
            log_.debug("PLAN", msg);
        }
    }
}
