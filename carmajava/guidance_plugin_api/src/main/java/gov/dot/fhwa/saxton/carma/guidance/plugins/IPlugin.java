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

import gov.dot.fhwa.saxton.carma.guidance.trajectory.Trajectory;
import gov.dot.fhwa.saxton.utils.Versionable;

/**
 * Interface describing the basic functionality of a CARMA Platform Guidance Plugin
 * <p>
 * The methods defined on this interface are used by other Guidance component elements to describe,
 * instantiate, control, and communicate with the plugin instances.
 * <p>
 * Plugins MUST implement this interface in their primary execution class otherwise the Guidance
 * PluginManager will not be able to locate the plugin on the classpath and it will not show up as
 * available for activation by the user.
 */
public interface IPlugin extends Versionable {

    // Lifecycle methods

    /**
     * Called when the plugin is first instantiated by the Guidance executor
     */
    void onInitialize();

    /**
     * Called after onInitialize() and any time the plugin resumes from a suspended state of execution.
     */
    void onResume();

    /**
     * Main execution loop fro the plugin. Should be where the plugin spends the majority of its
     * time while active.
     * <p>
     * Will be invoked by the PluginExecutor in a tight busy-loop. If the plugin needs to run at a
     * specific frequency it is the plugin's responsibility to insert the required timing logic.
     */
    void loop() throws InterruptedException;

    /**
     * Called before onTerminate() and any time the plugin is about to enter a state of suspended
     * execution
     */
    void onSuspend();

    /**
     * Called shortly before the plugin execution will terminate. Ensure that
     * any held resources are closed or otherwise released such that they do not leak.
     */
    void onTerminate();

    /**
     * Get the activation state of the plugin
     *
     * @return True if the user has commanded activation of the plugin, false o.w.
     */
    boolean getActivation();

    /**
     * Set the plugin activation value. Generally only used via the user interface at direct command
     * from the user.
     */
    void setActivation(boolean activation);

    /**
     * Get the availability state of the plugin.
     *
     * @return True if the plugin is relevant in the current driving situation, false o.w.
     */
    boolean getAvailability();

    // Activity methods

    /**
     * Execute the plugin's planning algorithm and generate maneuvers in the supplied trajectory if
     * possible.
     * 
     * @param traj The current partially planned Trajectory, which cannot be modified
     * @param expectedEntrySpeed The speed (in m/s) the vehicle is expected to have upon the start of the new trajectory
     */
    void planTrajectory(Trajectory traj, double expectedEntrySpeed);

    /**
     * Callback method to handle negotiation requests received from external or internal sources
     */
    void onReceiveNegotiationRequest();

    /**
     * Register an event listener to be called when the availability status of the IPlugin changes
     * @param availabilityListener
     */
    void registerAvailabilityListener(AvailabilityListener availabilityListener);
}
