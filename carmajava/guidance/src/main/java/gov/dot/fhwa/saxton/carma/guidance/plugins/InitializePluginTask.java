package gov.dot.fhwa.saxton.carma.guidance.plugins;

/**
 * Asynchronous task for Initializing a plugin
 */
public class InitializePluginTask implements Runnable {
    InitializePluginTask(IPlugin plugin, TaskCompletionCallback callback) {
        this.plugin = plugin;
        this.callback = callback;
    }

    @Override public void run() {
        plugin.onInitialize();
        callback.onComplete();
    }

    protected IPlugin plugin;
    protected TaskCompletionCallback callback;
}
