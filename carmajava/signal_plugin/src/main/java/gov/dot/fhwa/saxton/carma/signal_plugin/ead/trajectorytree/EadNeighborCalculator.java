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

import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.SignalPhase;
import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.utils.GlidepathApplicationContext;
import gov.dot.fhwa.saxton.carma.signal_plugin.asd.IntersectionData;
import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.IGlidepathAppConfig;
import gov.dot.fhwa.saxton.carma.signal_plugin.logger.ILogger;
import gov.dot.fhwa.saxton.carma.signal_plugin.logger.LoggerManager;

import java.util.ArrayList;
import java.util.List;

import static gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.Constants.MPS_TO_MPH;

/**
 * Calculates the viable neighbors for a node in the solution tree for the EAD model.
 */
public class EadNeighborCalculator implements INeighborCalculator {

    private class IntersectionHistory {
        int     id = 0;
        double  longestGreen = 0.0;
        double  longestYellow = 0.0;
        double  longestRed = 0.0;
    }

    protected final double                  DEFAULT_GREEN_DURATION = 20.0;  //sec
    protected final double                  DEFAULT_YELLOW_DURATION = 5.0;  //sec
    protected final double                  DEFAULT_RED_DURATION = 25.0;    //sec

    protected List<IntersectionData>        intersections_;
    protected int                           numInt_;
    protected double                        timeInc_;       // sec
    protected double                        speedInc_;      // m/s
    protected List<IntersectionHistory>     history_;
    protected double                        maxAccel_;      // m/s^2
    protected double                        speedLimit_;    // m/s
    protected double                        crawlingSpeed_; // m/s
    protected double                        stoppingLookAhead_; // time, sec, to test for imminent red violation
    protected double                        acceptableStopDist_; //max dist before bar it's acceptable to stop, m
    protected double                        timeBuffer_;    // time, sec, at end of green phase that we don't want to touch
    protected double                        debugThreshold_;//node distance beyond which we will turn on debug logging
    protected static ILogger                log_ = LoggerManager.getLogger(EadNeighborCalculator.class);


    public EadNeighborCalculator() {
        history_ = new ArrayList<>();

        //get config parameters
        IGlidepathAppConfig config = GlidepathApplicationContext.getInstance().getAppConfig();
        maxAccel_ = config.getDoubleDefaultValue("defaultAccel", 2.0);
        speedLimit_ = config.getMaximumSpeed(0.0) / MPS_TO_MPH;
        crawlingSpeed_ = config.getDoubleDefaultValue("crawlingSpeed", 5.0) / MPS_TO_MPH;
        acceptableStopDist_ = config.getDoubleDefaultValue("ead.acceptableStopDistance", 6.0);
        stoppingLookAhead_ = 5.0; //default
        timeBuffer_ = config.getDoubleDefaultValue("ead.timebuffer", 4.0);
        debugThreshold_ = config.getDoubleDefaultValue("ead.debugThreshold", -1.0);
    }


    @Override
    public void initialize(List<IntersectionData> intersections, int numIntersections, double timeIncrement,
                           double speedIncrement) {

        intersections_ = intersections;
        numInt_ = numIntersections;
        timeInc_ = timeIncrement;
        speedInc_ = speedIncrement;
        log_.debug("EAD", "initialize called with timeInc = " + timeIncrement + ", speedInc = " + speedIncrement);

        //ensure we will be thinking about stopping at least 2 nodes prior to the stop bar
        stoppingLookAhead_ = Math.max(stoppingLookAhead_, 2.01*timeIncrement);

        //This method will be called each time the EAD model replans, which will happen each time a visible signal
        // changes phases. This is an opportunity to learn more about its cycle timing, so grab whatever info is there.

        //loop through known intersections
        for (IntersectionData i : intersections) {
            boolean found = false;
            //if this intersection ID is already in the local history list then
            if (history_.size() > 0) {
                for (IntersectionHistory h : history_) {
                    if (i.intersectionId == h.id) {
                        //if its current phase data extends what we already know then record it
                        updateHistory(i, h);
                        found = true;
                        log_.debug("EAD", "initialize updating history for intersection " + h.id);
                        break;
                    }
                }
            }
            //else record current phase data in our history
            if (!found) {
                IntersectionHistory newHist = new IntersectionHistory();
                newHist.id = i.intersectionId;
                updateHistory(i, newHist);
                history_.add(newHist);
                log_.debug("EAD", "initialize adding new intersection " + newHist.id + " to history.");
            }
        }
    }


    @Override
    public List<Node> neighbors(Node node) {
        List<Node> neighbors = new ArrayList<>();

        //all valid neighbors will have their time one time increment larger than the input node's
        double curTime = node.getTimeAsDouble();
        double curDist = node.getDistanceAsDouble();
        double curSpeed = node.getSpeedAsDouble();
        double newTime = curTime + timeInc_;

        //determine viable speeds for neighbors, based on accel limit and other limitations
        // Lower limit is arbitrarily configured as crawling speed to avoid control problems at really low
        // speeds. We are only allowed to set target speeds below this value if we are coming to a stop or starting
        // out from a stop. Nodes with a speed of exactly zero are normal and acceptable, however (sitting
        // at a red light).

        //get candidate upper & lower speeds based on acceleration limits; we will create nodes at regular increments
        // between these
        double minSpeed = Math.max(curSpeed - maxAccel_*timeInc_, 0.0);
        double maxSpeed = Math.min(curSpeed + maxAccel_*timeInc_, speedLimit_);
        logDebug(curDist, "Generating neighbors of node " + node.toString() +
                    ". Initial minSpeed = " + minSpeed + ", maxSpeed = " + maxSpeed);

        List<Double> speeds = new ArrayList<>();
        boolean stopping = false;
        boolean starting = false;

        //if current speed > crawling, allow it to be a candidate for future nodes
        if (curSpeed >= crawlingSpeed_) {
            speeds.add(curSpeed);
        }

        //if we are at risk of running a red light in the next few sec (or are already stopped at one) then
        double endDist = curDist + stoppingLookAhead_*Math.max(curSpeed, crawlingSpeed_); //worst case
        boolean violation = signalViolation(curDist, endDist, curTime, curTime + stoppingLookAhead_);
        if (violation) {
            //indicate we need to be stopping
            stopping = true;
            logDebug(curDist, "We are presumably stopping. curSpeed = " + curSpeed);

        //else if current speed is less than crawling speed then
        }else if (curSpeed < crawlingSpeed_) {
            //indicate we are starting up
            starting = true;
            logDebug(curDist, "We are presumably starting up. curSpeed = " + curSpeed);
        }

        //if we are not in the process of stopping then
        if (!stopping) {
            //if we are starting up then
            if (starting) {
                //limit the minimum speed to a hair above current (force an acceleration)
                minSpeed = curSpeed + 0.1;
            }else {
                //we are at least at crawling speed, so limit the minimum speed to crawling
                minSpeed = Math.max(minSpeed, crawlingSpeed_);
            }

            //find viable speeds above current speed
            double speed = curSpeed + speedInc_;
            double prevMaxSpeed = curSpeed;
            while (speed <= maxSpeed) {
                speeds.add(speed);
                prevMaxSpeed = speed;
                speed += speedInc_;
            }

            //if acceleration allows us to reach speed limit, ensure it is one of the speeds being evaluated
            // (subtract 0.01 to account for possible round-off error when matching maxSpeed; subtract 0.25
            // on the right side to avoid checking two speeds that are really close together)
            if (maxSpeed >= speedLimit_ - 0.01  &&  prevMaxSpeed < speedLimit_ - 0.25) {
                speeds.add(speedLimit_);
            }
        }

        //if we are not starting up then
        if (!starting) {
            //look at all lower speeds
            double speed = curSpeed - speedInc_;
            double prevMinSpeed = curSpeed;
            while (speed >= minSpeed) {
                speeds.add(speed);
                prevMinSpeed = speed;
                speed -= speedInc_;
            }

            //if we are allowed to go to full stop then ensure it is one of the speeds being evaluated
            if (minSpeed == 0.0  &&  (prevMinSpeed > 0.0  ||  curSpeed == 0.0)) {
                speeds.add(0.0);
            }
        }

        //loop on the reachable speed increments
        if (speeds.size() > 0) {
            for (double s : speeds) {
                //compute the distance to be travelled going from the current speed to that new speed
                double newDist = curDist + 0.5 * (curSpeed + s) * timeInc_;

                //if the new downtrack distance is  going to violate a red signal then don't allow this to be a neighbor
                if (signalViolation(curDist, newDist, curTime, newTime)) {
                    logDebug(curDist, "Speed " + s + " will give a signal violation. Throwing it away.");
                    continue;
                }

                //if the new speed is 0 and we are somewhat close to the stop bar (appear to be stopping at red), but
                // uncomfortably far away for stopping at a light then ignore this possible neighbor
                // (we need to only test when dtsb is kinda close to stop bar, because we may be legitimately
                // stopped farther away, waiting to start an experiment)
                double dtsb = distToIntersection(currentIntersectionIndex(curDist), curDist);
                if (s < 0.1 && dtsb > acceptableStopDist_ && dtsb < 6.0 * acceptableStopDist_) {
                    logDebug(curDist, "Speed " + s + " will stop too soon. Throwing it away.");
                    continue;
                }

                //add a node to the neighbor list to represent this new situation
                Node newNode = new Node(newDist, newTime, s);
                neighbors.add(newNode);
            }
        }
        //log_.debug("EAD", "returning " + neighbors.size() + " neighbors.");

        return neighbors;
    }

    ////////////////////

    /**
     * Stored available phase duration info from the current intersection data into our internal history record
     * @param i - updated data on a known intersection
     * @param h - historical record of intersection phase durations
     */
    private void updateHistory(IntersectionData i, IntersectionHistory h) {
        switch (i.currentPhase) {
            case GREEN:
                h.longestGreen  = Math.max(h.longestGreen,  i.timeToNextPhase);
                h.longestYellow = Math.max(h.longestYellow, i.timeToThirdPhase);
                break;
            case YELLOW:
                h.longestYellow = Math.max(h.longestYellow, i.timeToNextPhase);
                h.longestRed    = Math.max(h.longestRed,    i.timeToThirdPhase);
                break;
            case RED:
                h.longestRed    = Math.max(h.longestRed,    i.timeToNextPhase);
                h.longestGreen  = Math.max(h.longestGreen,  i.timeToThirdPhase);
                break;
            default:
                //do nothing - this is a normal condition, especially for intersections beyond the nearest
        }
    }



    /**
     * Determines if the vehicle would violate signal laws when travelling from the start to final conditions specified
     * @param startDist - starting distance downtrack of plan start, m
     * @param endDist - final distance downtrack of plan start, m
     * @param startTime - starting time since beginning of plan, sec
     * @param endTime - final time since start of plan, sec
     * @return true if a violation would occur (runs a red light) while moving from start to end
     */
    private boolean signalViolation(double startDist, double endDist, double startTime, double endTime) {
        //log_.debug("EAD", "Entering signalViolation: startDist = " + startDist + ", endDist = "
        //            + endDist + ", startTime = " + startTime + ", endTime = " + endTime);

        //determine which intersection we are approaching [currentIntersectionIndex]
        int currentInt = currentIntersectionIndex(startDist);
        if (currentInt == -1) {
            return false;
        }

        //if the travel from start to end is not going to cross the stop bar then
        double distToInt = distToIntersection(currentInt, endDist);
        if (distToInt > 0.0) {
            return false;
        }

        //figure out exactly when we will cross the stop bar
        // Note: distToIntersection here will give the distance from start of plan to the stop bar, which is
        // the location of interest where we will cross; use this since all other distances are relative to plan start
        double interpFactor = (distToIntersection(currentInt, 0.0) - startDist) / (endDist - startDist);
        double crossingTime = startTime + interpFactor*(endTime - startTime);
        //log_.debug("EAD", "Stop bar will be crossed: interpFactor = " + interpFactor
        //            + ", crossingTime = " + crossingTime);

        //to account for uncertainties in the vehicle's dynamic response to speed command changes, we need
        // a little wiggle room, so don't want to cross the bar just as signal is changing color; we want
        // to avoid red at crossing time +/- the specified buffer
        boolean redIfEarly = phaseAtTime(currentInt, crossingTime - timeBuffer_) == SignalPhase.RED;
        boolean redIfLate  = phaseAtTime(currentInt, crossingTime + timeBuffer_) == SignalPhase.RED;
        return redIfEarly || redIfLate;
    }


    /**
     * Determines which intersection is currently in front of us.
     * @param dist - the specified distance downtrack from start of current plan, m
     * @return index to the current intersection, or -1 if we are past all known intersections
     */
    private int currentIntersectionIndex(double dist) {
        int current;
        //Assumes the intersections_ attribute is sorted from nearest to farthest
        //Note that the intersection's dtsb and roughDist are measured from the stop bar outward
        if (intersections_.size() > 0) {
            for (current = 0; current < intersections_.size(); ++current) {
                if (distToIntersection(current, dist) > 0.0) {
                    return current;
                }
            }
        }

        return -1;
    }


    /**
     * Computes the distance to the stop bar of the intersection with the given internal list index
     * @param index - index in the intersections_ list
     * @param distDowntrack - current distance downtrack from the plan start, m
     * @return distance, m (or infinity if the index is invalid)
     */
    private double distToIntersection(int index, double distDowntrack) {
        if (index < 0) {
            return Double.POSITIVE_INFINITY;
        }
        //all of the distances in intersections_ are relative to plan starting point, established when the
        // initialize() method was called, but they are not updated after that
        IntersectionData i = intersections_.get(index);
        double iDist = i.dtsb;
        if (iDist <= 0.0) {
            iDist = 0.01*(double)i.roughDist; //convert from cm to m
        }
        return iDist - distDowntrack;
    }


    /**
     * Returns the signal phase to be expected at the specified intersection at the specified time
     * @param intersectionIndex - index in the intersections_ list of the intersection in question
     * @param futureTime - time since beginning of plan, sec
     * @return expected phase at that time
     */
    protected SignalPhase phaseAtTime(int intersectionIndex, double futureTime) {
        IntersectionData i = intersections_.get(intersectionIndex);

        if (futureTime <= i.timeToNextPhase) {
            return i.currentPhase;
        }

        double cycleTime = i.timeToNextPhase;
        SignalPhase phase = i.currentPhase;
        IntersectionHistory h = findHistoricalData(intersectionIndex);
        if (h != null) {
            do {
                phase = phase.next();
                if (phase == SignalPhase.NONE) {
                    phase =  phase.next();
                }
                switch(phase) {
                    case GREEN:
                        cycleTime += h.longestGreen > 0.0 ? h.longestGreen : DEFAULT_GREEN_DURATION;
                        break;
                    case YELLOW:
                        cycleTime += h.longestYellow > 0.0 ? h.longestYellow : DEFAULT_YELLOW_DURATION;
                        break;
                    case RED:
                        cycleTime += h.longestRed > 0.0 ? h.longestRed : DEFAULT_RED_DURATION;
                        break;
                    default:
                        log_.warn("EAD", "phaseAtTime attempted to use next phase of " + phase.toString());
                }
            } while(cycleTime < futureTime);
        }

        return phase;
    }


    /**
     * Retrieves the historical data we have on the specified intersection.
     * @param index - index into the intersections_ list for the specified intersection
     * @return history record for this intersection
     */
    private IntersectionHistory findHistoricalData(int index) {
        for (IntersectionHistory h : history_) {
            if (h.id == intersections_.get(index).intersectionId) {
                return h;
            }
        }
        log_.warn("EAD", "findHistoricalData failed to find any for index = " + index + ", id = "
                    + intersections_.get(index).intersectionId);

        return null;
    }

    private void logDebug(double distance, String msg) {
        if (debugThreshold_ >= 0.0  &&  distance >= debugThreshold_) {
            log_.debug("PLAN", msg);
        }
    }
}
