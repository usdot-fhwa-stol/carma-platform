package gov.dot.fhwa.saxton.carma.guidance.plugins;

public class SuspendPluginTask implements Runnable {
    protected IPlugin plugin;
    protected TaskCompletionCallback callback;

    SuspendPluginTask(IPlugin plugin, TaskCompletionCallback callback) {
        this.plugin = plugin;
        this.callback = callback;
    }

    @Override public void run() {
        plugin.onSuspend();
        callback.onComplete();
    }
}
