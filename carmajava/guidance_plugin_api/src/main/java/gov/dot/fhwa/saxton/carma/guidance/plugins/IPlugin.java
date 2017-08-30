/*
 * TODO: Copyright (C) 2017 LEIDOS.
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

/**
 * Interface describing the basic functionality of a CARMA Platform Guidance Plugin
 *
 * The methods defined on this interface are used by other Guidance component elements to describe,
 * instantiate, control, and communicate with the plugin instances.
 *
 * Plugins MUST implement this interface in their primary execution class otherwise the Guidance
 * PluginManager will not be able to locate the plugin on the classpath and it will not show up as
 * available for activation by the user.
 */
public interface IPlugin {
    // Metadata methods
    /**
     * Get the name of the plugin instance, generally the name of the algorithm or the application.
     */
    String getName();

    /**
     * Get a String representation of the plugin's version. getName() and getVersionId() should
     * uniquely identify a particular deployment of a plugin.
     */
    String getVersionId();

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
     *
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
     */
    void planTrajectory();

    /**
     * Callback method to handle negotiation requests received from external or internal sources
     */
    void onReceiveNegotiationRequest();
}
