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

package gov.dot.fhwa.saxton.carma.guidance.plugins;

import gov.dot.fhwa.saxton.carma.guidance.arbitrator.TrajectoryPlanningResponse;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.IPubSubService;
import gov.dot.fhwa.saxton.carma.guidance.trajectory.Trajectory;
import gov.dot.fhwa.saxton.carma.guidance.util.ILogger;
import gov.dot.fhwa.saxton.carma.guidance.util.LoggerManager;
import gov.dot.fhwa.saxton.utils.ComponentVersion;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.*;

/**
 * Abstract base class for plugins
 * <p>
 * Provides the basic getters and setters so that implementors don't have to deal with that
 */
public abstract class AbstractPlugin implements IPlugin {
	protected ComponentVersion version = new ComponentVersion();
    protected IPubSubService pubSubService;
    protected PluginServiceLocator pluginServiceLocator;
    protected ILogger log;

    // Private fields so that extendees can't access them
    private AtomicBoolean activation = new AtomicBoolean(false);
    private AtomicBoolean availability = new AtomicBoolean(false);
    private List<AvailabilityListener> availabilityListeners = new ArrayList<>();

    public AbstractPlugin(PluginServiceLocator pluginServiceLocator) {
        this.pluginServiceLocator = pluginServiceLocator;
        this.pubSubService = pluginServiceLocator.getPubSubService();
        
        // Work around to get the name of the extending class instead of "AbstractPlugin"
        this.log = LoggerManager.getLogger(this.getClass().getCanonicalName());
    }

    @Override public boolean getActivation() {
        return activation.get();
    }

    @Override public void setActivation(boolean activation) {
        this.activation.set(activation);
        if (!this.activation.get()) {
            setAvailability(false);
        }
    }

    @Override public final boolean getAvailability() {
        return availability.get();
    }

    @Override public TrajectoryPlanningResponse planTrajectory(Trajectory traj, double expectedEntrySpeed) {
        throw new UnsupportedOperationException();
    }

    @Override public void onReceiveNegotiationRequest(String strategy) {
        throw new UnsupportedOperationException();
    }

    @Override
    public final void registerAvailabilityListener(AvailabilityListener availabilityListener) {
        availabilityListeners.add(availabilityListener);
    }
    
    @Override
    public ComponentVersion getVersionInfo() {
    	return version;
    }

    /**
     * Use this method to change the plugin's availability status
     *
     * Will automatically ensure all availability listeners are notified
     *
     * @param availability
     */
    protected final void setAvailability(boolean availability) {
        if (this.availability.get() != availability) {
            this.availability.set(availability);
            log.debug("Availability Listeners " + availabilityListeners.size());
            for (AvailabilityListener el : availabilityListeners) {
                el.onAvailabilityChange(this, availability);
            }
        }
    }
}
