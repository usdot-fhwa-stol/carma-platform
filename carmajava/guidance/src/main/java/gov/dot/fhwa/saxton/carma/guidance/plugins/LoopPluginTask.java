package gov.dot.fhwa.saxton.carma.guidance.plugins;

/**
 * Asynchronous task for calling a plug-in's loop function repeatedly until the thread is
 * interrupted.
 */
public class LoopPluginTask implements Runnable {
    protected TaskCompletionCallback callback;
    protected IPlugin plugin;

    LoopPluginTask(IPlugin plugin, TaskCompletionCallback callback) {
        this.plugin = plugin;
        this.callback = callback;
    }

    @Override public void run() {
        // Invoke the complete callback early since there is no LOOPED state only LOOPING
        callback.onComplete();
        while (!Thread.currentThread().isInterrupted()) {
            try {
                plugin.loop();
            } catch (InterruptedException e) {
                // Rethrow the interruption
                Thread.currentThread().interrupt();
            }
        }
    }
}
