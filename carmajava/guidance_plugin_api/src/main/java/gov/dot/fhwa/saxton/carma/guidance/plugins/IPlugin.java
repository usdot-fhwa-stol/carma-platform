package gov.dot.fhwa.saxton.carma.guidance.plugins;

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
    void onCreate();

    /**
     * Called after onCreate() and any time the plugin resumes from a suspended state of execution.
     */
    void onResume();

    /**
     * Called before onDestroy() and any time the plugin is about to enter a state of suspended
     * execution
     */
    void onSuspend();

    /**
     * Called shortly before the plugin will be destroyed and execution will terminate. Ensure that
     * any held resources are closed or otherwise released such that they do not leak.
     */
    void onDestroy();

    /**
     * Set the plugin activation value. Generally only used via the user interface at direct command
     * from the user.
     */
    void setActivation(boolean activation);

    /**
     * Get the activation state of the plugin
     * @return True if the user has commanded activation of the plugin, false o.w.
     */
    boolean getActivation();

    /**
     * Get the availability state of the plugin.
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
