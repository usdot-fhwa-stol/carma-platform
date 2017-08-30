package gov.dot.fhwa.saxton.carma.guidance.plugins;

import org.apache.commons.logging.Log;

import java.util.HashMap;
import java.util.Map;

/**
 * Class responsible for coordinating the execution of multiple plugins
 *
 * Creates {@link PluginLifecycleHandler} instances for each plugin then provides access to them
 * based on the plugin name and plugin version values.
 */
public class PluginExecutor {

    PluginExecutor(Log log) {
        this.log = log;
    }

    /**
     * Submit a plugin for management by the PluginExecutor
     *
     * This plugin will be indexed by it's name and versionId fields, which are assumed to be unique.
     * In the event these are not unique, a duplicate submission will result in the older submission
     * being replaced.
     *
     * Note that this method does not begin the execution, it simply sets up the threading framework
     * to support that. To begin the exection call {@link PluginExecutor#initializePlugin(String, String)}
     * and {@link PluginExecutor#resumePlugin(String, String)}.
     *
     * @param plugin The plugin instance to begin tracking.
     */
    public void submitPlugin(IPlugin plugin) {
        PluginLifecycleHandler handler = new PluginLifecycleHandler(plugin, log);
        lifecycleHandlers.put(plugin.getName() + plugin.getVersionId(), handler);
    }

    /**
     * Asynchronously invoke a plugin's {@link IPlugin#onInitialize()} method
     *
     * Locates the {@link PluginLifecycleHandler} instance associated with the specified plugin
     * and causes it to call the plugins initialization behavior.
     *
     * @param pluginName The string plugin name as reported by {@link IPlugin#getName()}
     * @param pluginVersion The string plugin version as reported by {@link IPlugin#getVersionId()}
     */
    public void initializePlugin(String pluginName, String pluginVersion) {
        lifecycleHandlers.get(pluginName + pluginVersion).initialize();
    }

    /**
     * Asynchronously invoke a plugin's {@link IPlugin#onResume()} ()} method, then indefinitely
     * loop over that plugin's {@link IPlugin#loop()} method.
     *
     * Locates the {@link PluginLifecycleHandler} instance associated with the specified plugin
     * and causes it to call the plugins resume and loop behavior.
     *
     * @param pluginName The string plugin name as reported by {@link IPlugin#getName()}
     * @param pluginVersion The string plugin version as reported by {@link IPlugin#getVersionId()}
     */
    public void resumePlugin(String pluginName, String pluginVersion) {
        lifecycleHandlers.get(pluginName + pluginVersion).resume();
    }

    /**
     * Asynchronously invoke a plugin's {@link IPlugin#onSuspend()} ()} method
     *
     * Locates the {@link PluginLifecycleHandler} instance associated with the specified plugin
     * and causes it to call the plugins suspend behavior
     *
     * @param pluginName The string plugin name as reported by {@link IPlugin#getName()}
     * @param pluginVersion The string plugin version as reported by {@link IPlugin#getVersionId()}
     */
    public void suspendPlugin(String pluginName, String pluginVersion) {
        lifecycleHandlers.get(pluginName + pluginVersion).suspend();
    }

    /**
     * Asynchronously invoke a plugin's {@link IPlugin#onTerminate()} method
     *
     * Locates the {@link PluginLifecycleHandler} instance associated with the specified plugin
     * and causes it to call the plugins terminate behavior. Then deletes the associated
     * PluginLifeCycleHandler
     *
     * @param pluginName The string plugin name as reported by {@link IPlugin#getName()}
     * @param pluginVersion The string plugin version as reported by {@link IPlugin#getVersionId()}
     */
    public void terminatePlugin(String pluginName, String pluginVersion) {
        lifecycleHandlers.get(pluginName + pluginVersion).terminate();
        lifecycleHandlers.remove(pluginName + pluginVersion);
    }

    protected Map<String, PluginLifecycleHandler> lifecycleHandlers = new HashMap<>();
    protected Log log;
    protected PluginServiceLocator pluginServiceLocator;
}
