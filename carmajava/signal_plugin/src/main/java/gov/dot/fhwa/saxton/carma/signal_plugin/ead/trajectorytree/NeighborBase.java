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
import gov.dot.fhwa.saxton.carma.signal_plugin.asd.IntersectionData;
import gov.dot.fhwa.saxton.carma.signal_plugin.ead.INodeCollisionChecker;

import java.util.Arrays;
import java.util.List;

import static gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.SignalPhase.NONE;

public abstract class NeighborBase implements INeighborCalculator {

    protected class IntersectionHistory {
        int     id = 0;
        double  longestGreen = 0.0;
        double  longestYellow = 0.0;
        double  longestRed = 0.0;
        
        @Override
        public String toString() {
            return "IntersectionHistory [id=" + id + ", longestGreen=" + longestGreen + ", longestYellow="
                    + longestYellow + ", longestRed=" + longestRed + "]";
        }
    }

    protected class SignalState {
        SignalPhase phase = NONE;
        double      timeRemaining = 0.0;
    }

    protected final double                  DEFAULT_GREEN_DURATION = 27.0;  //sec
    protected final double                  DEFAULT_YELLOW_DURATION = 3.0;  //sec
    protected final double                  DEFAULT_RED_DURATION = 30.0;    //sec

    protected List<IntersectionData>        intersections_;
    protected int                           numInt_;
    protected double                        timeInc_;       // sec
    protected double                        speedInc_;      // m/s
    protected List<IntersectionHistory>     history_;
    protected double                        maxAccel_;      // m/s^2
    protected double                        speedLimit_;    // m/s
    protected double                        crawlingSpeed_; // m/s
    protected double                        timeBuffer_;    // time, sec, at end of green phase that we don't want to touch
    protected double                        operSpeed_;     //desired vehicle speed if no signal constraints, m/s

    protected INodeCollisionChecker         collisionChecker_;

    protected double                        planningStartTime_; // Starting planning time in seconds.
    protected double                        planningStartDowntrack_; // Starting planning downtrack distance 

    /**
     * This will only be called when we are on the map of an intersection, so input intersections is
     * guaranteed to have at least one member.  They will be ordered from nearest to farthest.
     */
    @Override
    public void initialize(List<IntersectionData> intersections, int numIntersections, double timeIncrement,
                           double speedIncrement, INodeCollisionChecker collisionChecker, double planningStartTime, double planningStartDowntrack) {
        intersections_ = intersections;
        numInt_ = numIntersections;
        timeInc_ = timeIncrement;
        speedInc_ = speedIncrement;
        collisionChecker_ = collisionChecker;
        planningStartTime_ = planningStartTime;
        planningStartDowntrack_ = planningStartDowntrack;

        //This method will be called each time the EAD model replans, which will happen each time a visible signal
        // changes phases. This is an opportunity to learn more about its cycle timing, so grab whatever info is there.

        //loop through known intersections
        for (IntersectionData i : intersections) {
            boolean found = false;
            //if this intersection ID is already in the local history list then
            if (history_.size() > 0) {
                for (IntersectionHistory h : history_) {
                    if (i.getIntersectionId() == h.id) {
                        //if its current phase data extends what we already know then record it
                        updateHistory(i, h);
                        found = true;
                        break;
                    }
                }
            }
            //else record current phase data in our history
            if (!found) {
                IntersectionHistory newHist = new IntersectionHistory();
                newHist.id = i.getIntersectionId();
                updateHistory(i, newHist);
                history_.add(newHist);
            }
        }
    }


    @Override
    public void setOperatingSpeed(double os) {
        operSpeed_ = os;
    }


    @Override
    public abstract List<Node> neighbors(Node node);


    /**
     * Retrieves the historical data we have on the specified intersection.
     * @param index - index into the intersections_ list for the specified intersection
     * @return history record for this intersection
     */
    public IntersectionHistory getHistoricalData(int index) {
        for (IntersectionHistory h : history_) {
            if (h.id == intersections_.get(index).getIntersectionId()) {
                return h;
            }
        }
        return null;
    }


    ////////////////////


    /**
     * Stored available phase duration info from the current intersection data into our internal history record
     * @param i - updated data on a known intersection
     * @param h - historical record of intersection phase durations
     */
    protected void updateHistory(IntersectionData i, IntersectionHistory h) {
        switch (i.getCurrentPhase()) {
            case GREEN:
                h.longestGreen  = Math.max(h.longestGreen,  i.getTimeToNextPhase());
                break;
            case YELLOW:
                h.longestYellow = Math.max(h.longestYellow, i.getTimeToNextPhase());
                break;
            case RED:
                h.longestRed    = Math.max(h.longestRed,    i.getTimeToNextPhase());
                break;
            default:
                //do nothing - this is a normal condition, especially for intersections beyond the nearest
        }
    }


    /**
     * Determines which intersection is currently in front of us.
     * @param dist - the specified distance downtrack from start of current plan, m
     * @return index to the current intersection, or -1 if we are past all known intersections
     */
    protected int currentIntersectionIndex(double dist) {
        //Assumes the intersections_ attribute is sorted from nearest to farthest
        //Note that the intersection's dtsb and roughDist are measured from the stop bar outward
        for (int current = 0; current < intersections_.size(); ++current) {
            if (distToIntersection(current, dist) >= 0.0) {
                return current;
            }
        }

        return -1;
    }


    /**
     * Computes the distance to the stop bar of the intersection with the given internal list index
     * @param index - index in the intersections_ list
     * @param startLoc - current distance downtrack from the plan start, m
     * @return distance, m (or infinity if the index is invalid)
     */
    protected double distToIntersection(int index, double startLoc) {
        if (index < 0) {
            return Double.POSITIVE_INFINITY;
        }
        //all of the locations in intersections_ are distances from plan starting point, established when the
        // initialize() method was called, but they are not updated after that
        IntersectionData i = intersections_.get(index);
        double iDist = i.getDtsb();

        //System.out.println("BestDTSB for int: " + i.intersectionId + " index: " + index + " dtsb: " + i.bestDTSB() + " startLoc: " + startLoc);
        return Node.roundToDistUnits(i.bestDTSB() - startLoc);
    }


    /**
     * Returns the signal phase & time remaining in phase to be expected at the specified intersection at the specified time
     * @param intersectionIndex - index in the intersections_ list of the intersection in question
     * @param futureTime - time since beginning of plan, sec
     * @return expected signal state (phase and time remaining in that phase) at futureTime
     */
    protected SignalState phaseAtTime(int intersectionIndex, double futureTime) {
        SignalState result = new SignalState();
        IntersectionData i = intersections_.get(intersectionIndex);

        if (futureTime <= i.getTimeToNextPhase()) {
            result.phase = i.getCurrentPhase();
            result.timeRemaining = i.getTimeToNextPhase() - futureTime;
            return result;
        }

        double cycleTime = i.getTimeToNextPhase();
        SignalPhase phase = i.getCurrentPhase();
        IntersectionHistory h = getHistoricalData(intersectionIndex);
        if (h != null) {
            do {
                phase = phase.next();
                if (phase == NONE) {
                    phase =  phase.next();
                }
                cycleTime += phaseDuration(intersectionIndex, phase);
            } while(cycleTime < futureTime);
        }

        result.phase = phase;
        result.timeRemaining = cycleTime - futureTime;
        return result;
    }


    /**
     * Determines the duration of the requested phase in the given intersection, to the best of our current knowledge
     * @param intersectionIndex - index of the intersection in question
     * @param phase - phase in question
     * @return total phase duration, sec
     */
    protected double phaseDuration(int intersectionIndex, SignalPhase phase) {
        IntersectionHistory h = getHistoricalData(intersectionIndex);
        
        switch(phase) {
        //add max in those cases to make sure it did not return extremely small value
            case GREEN:
                return Math.max(h.longestGreen, DEFAULT_GREEN_DURATION);
            case YELLOW:
                return Math.max(h.longestYellow, DEFAULT_YELLOW_DURATION);
            case RED:
                return Math.max(h.longestRed, DEFAULT_RED_DURATION);
            default:
                //would be nice to log a warning here, but base class doesn't have a log object
                return -1.0;
        }
    }
    
    /**
     * Returns a boolean flag to indicate if the given node pair has any conflict with detected NCV 
     * @param startNode
     * @param endNode
     * @return true if collision detected
     */
    protected boolean hasConflict(Node startNode, Node endNode) {
        return this.collisionChecker_.hasCollision(Arrays.asList(startNode, endNode), planningStartTime_, planningStartDowntrack_);
    }
}
