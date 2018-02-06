/*
 * Copyright (C) 2017 LEIDOS.
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

package gov.dot.fhwa.saxton.carma.guidance.maneuvers;

import gov.dot.fhwa.saxton.carma.guidance.IGuidanceCommands;
import gov.dot.fhwa.saxton.carma.guidance.plugins.IPlugin;
import gov.dot.fhwa.saxton.carma.guidance.util.ILogger;
import gov.dot.fhwa.saxton.carma.guidance.util.LoggerManager;

/**
 * Base class for all simple maneuver objects.
 */
public abstract class ManeuverBase implements ISimpleManeuver {

    protected double                            startDist_ = -1.0; // m
    protected double                            endDist_ = -1.0;// m
    protected IManeuverInputs                   inputs_;
    protected IGuidanceCommands                 commands_;
    protected ILogger                           log_ = LoggerManager.getLogger();
    protected final IPlugin                     planner_;

    /**
     * Constructs a Maneuver and sets the planner's name
     * @param planner the plugin which planned this maneuver
     */
    public ManeuverBase(IPlugin planner) {
        this.planner_ = planner;
    }

    /**
     * Provides the common planning capability that all maneuvers will need. Concrete maneuver classes
     * will need to provide their own plan() methods to fill in the details and execute this one first.
     */
    @Override
    public void plan(IManeuverInputs inputs, IGuidanceCommands commands, double startDist) throws IllegalStateException {
        inputs_ = inputs;
        commands_ = commands;
        startDist_ = startDist;
    }

    /**
     * Provides the common planning capability that all maneuvers will need. Concrete maneuver classes
     * will need to provide their own plan() methods to fill in the details and execute this one first.
     */
    @Override
    public double planToTargetDistance(IManeuverInputs inputs, IGuidanceCommands commands, double startDist, double endDist) throws IllegalStateException {
        inputs_ = inputs;
        commands_ = commands;
        startDist_ = startDist;
        return Double.NaN;
    }

    @Override
    public abstract boolean canPlan(IManeuverInputs inputs, double startDist, double endDist);
    
    public abstract boolean executeTimeStep() throws IllegalStateException;

    @Override
    public double getStartDistance() {
        return startDist_;
    }


    @Override
    public double getEndDistance() {
        return endDist_;
    }
    
    @Override
    public IPlugin getPlanner() {
        return planner_;
    }

    /**
     * Verifies that the vehicle is between the specified start & end locations for this maneuver
     * @throws IllegalStateException if we are outside the allowable region
     */
    protected void verifyLocation() throws IllegalStateException {
        double currentLocation = inputs_.getDistanceFromRouteStart();
        if (currentLocation < startDist_  ||  currentLocation > endDist_) {
            throw new IllegalStateException("Maneuver attempted to execute at distance " + currentLocation
                                            + ". Maneuver start dist = " + startDist_ + ", end dist = " + endDist_);
        }
    }
}
