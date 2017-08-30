package gov.dot.fhwa.saxton.carma.guidance.plugins;

/**
 * Asynchronous task for invoking a plugin's termination behavior
 */
public class TerminatePluginTask implements Runnable {
    protected TaskCompletionCallback callback;
    protected IPlugin plugin;

    TerminatePluginTask(IPlugin plugin, TaskCompletionCallback callback) {
        this.plugin = plugin;
        this.callback = callback;
    }

    @Override public void run() {
        plugin.onTerminate();
        callback.onComplete();
    }
}
