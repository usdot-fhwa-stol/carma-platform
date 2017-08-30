package gov.dot.fhwa.saxton.carma.guidance.plugins;

import org.apache.commons.logging.Log;

import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.atomic.AtomicReference;

/**
 * Class responsible for ensuring valid lifecycle state transitions for IPlugin instances. Manages
 * a single-worker task queue to ensure that all actions are done as asynchronously as possible.
 * <p>
 * State transitions are added to this queue as they are requested.
 */
public class PluginLifecycleHandler {

    protected final BlockingQueue<Runnable> tasks; // Task queue
    protected final IPlugin plugin; // Executed plugin
    protected Thread t; // Worker thread
    protected AtomicReference<PluginState> state = new AtomicReference<>(PluginState.UNINITIALIZED);
        // Current state, thread safe
    protected Log log;

    PluginLifecycleHandler(IPlugin plugin, Log log) {
        this.tasks = new LinkedBlockingQueue<>();
        this.plugin = plugin;
        this.log = log;
    }

    /**
     * Private helper method for actually performing plugin initialization
     */
    private void doInitialize() {
        log.info("Initializing " + plugin.getName() + ":" + plugin.getVersionId());
        state.set(PluginState.INITIALIZING);
        tasks.add(new InitializePluginTask(plugin, new TaskCompletionCallback() {
            @Override public void onComplete() {
                state.set(PluginState.INITIALIZED);
            }
        }));

        t = new Thread(new PluginWorker(tasks));
        t.start();
    }

    /**
     * Initialize the plugin.
     * <p>
     * Only valid if the plugin is in the UNITIALIZED state as set by this PluginLifeCycleHandler's
     * creation. A new plugin instance and new corresponding PluginLifeCycleHandler must be created
     * to return the plugin to this state. Throws an illegal state exception if called in any other
     * state.
     *
     * @throws IllegalStateException
     */
    public void initialize() {
        switch (state.get()) {
            case UNINITIALIZED: {
                doInitialize();
                break;
            }
            case INITIALIZING:
                // INTENTIONAL FALL THROUGH
            case INITIALIZED:
                // INTENTIONAL FALL THROUGH
            case RESUMING:
                // INTENTIONAL FALL THROUGH
            case RESUMED:
                // INTENTIONAL FALL THROUGH
            case LOOPING:
                // INTENTIONAL FALL THROUGH
            case SUSPENDING:
                // INTENTIONAL FALL THROUGH
            case SUSPENDED:
                // INTENTIONAL FALL THROUGH
            case DESTROYING:
                // INTENTIONAL FALL THROUGH
            case DESTROYED:
                // INTENTIONAL FALL THROUGH
            default:
                throw new IllegalStateException();
        }
    }

    /**
     * Private helper method for actually performing the suspend operation
     */
    private void doResume() {
        log.info("Resuming " + plugin.getName() + ":" + plugin.getVersionId());
        tasks.add(new ResumePluginTask(plugin, new TaskCompletionCallback() {
            @Override public void onComplete() {
                state.set(PluginState.RESUMED);
            }
        }));

        // After resuming we always return to looping
        log.info("Looping " + plugin.getName() + ":" + plugin.getVersionId());
        tasks.add(new LoopPluginTask(plugin, new TaskCompletionCallback() {
            @Override public void onComplete() {
                state.set(PluginState.LOOPING);
            }
        }));
    }

    /**
     * Resume the IPlugin's activity.
     * <p>
     * Must only be called if the plugin is in INITIALIZED, SUSPENDING, or SUSPENDED state. As set by
     * the {@link PluginLifecycleHandler#initialize()} or {@link PluginLifecycleHandler#suspend()}
     * method. Throws an IllegalStateException otherwise.
     *
     * @throws IllegalStateException
     */
    public void resume() {
        switch (state.get()) {
            case UNINITIALIZED:
                throw new IllegalStateException();
                // INTENTIONAL FALL THROUGH
            case INITIALIZING:
            case INITIALIZED: {
                doResume();
                break;
            }
            case RESUMING:
                // INTENTIONAL FALL THROUGH
            case RESUMED:
                // INTENTIONAL FALL THROUGH
            case LOOPING:
                throw new IllegalStateException();
            case SUSPENDING:
                // INTENTIONAL FALL THROUGH
            case SUSPENDED: {
                doResume();
                break;
            }
            case DESTROYING:
                // INTENTIONAL FALL THROUGH
            case DESTROYED:
                // INTENTIONAL FALL THROUGH
            default:
                throw new IllegalStateException();
        }
    }

    /**
     * Private helper method for actually performing the suspend operation
     */
    private void doSuspend() {
        t.interrupt();
        tasks.clear();
        state.set(PluginState.SUSPENDING);
        tasks.add(new SuspendPluginTask(plugin, new TaskCompletionCallback() {
            @Override public void onComplete() {
                state.set(PluginState.SUSPENDED);
            }
        }));
        t = new Thread(new PluginWorker(tasks));
        t.start();
    }

    /**
     * Suspend the IPlugin instance.
     * <p>
     * Must only be called if the plugin is in RESUMED or LOOPING state, set by the
     * {@link PluginLifecycleHandler#resume()} method. Otherwise, will throw an IllegalStateException
     *
     * @throws IllegalStateException
     */
    public void suspend() {
        switch (state.get()) {
            case UNINITIALIZED:
                // INTENTIONAL FALL THROUGH
            case INITIALIZING:
                // INTENTIONAL FALL THROUGH
            case INITIALIZED:
                // INTENTIONAL FALL THROUGH
            case RESUMING:
                // INTENTIONAL FALL THROUGH
                throw new IllegalStateException();
            case RESUMED:
                // INTENTIONAL FALL THROUGH
            case LOOPING: {
                doSuspend();
                break;
            }
            case SUSPENDING:
                // INTENTIONAL FALL THROUGH
            case SUSPENDED:
                // INTENTIONAL FALL THROUGH
            case DESTROYING:
                // INTENTIONAL FALL THROUGH
            case DESTROYED:
                // INTENTIONAL FALL THROUGH
            default:
                throw new IllegalStateException();
        }
    }

    /**
     * Private helper method to actually execute the destruction of the IPlugin. Used to clean up
     * the switch/case statement below.
     */
    private void doTerminate() {
        t.interrupt();
        tasks.clear();
        state.set(PluginState.DESTROYING);
        tasks.add(new TerminatePluginTask(plugin, new TaskCompletionCallback() {
            @Override public void onComplete() {
                state.set(PluginState.DESTROYED);
            }
        }));

        t = null;
    }

    /**
     * Asynchronously trigger the shutdown of the IPlugin instance. Must only be called after first
     * suspending the IPlugin via the {@link PluginLifecycleHandler#suspend()} method.
     *
     * @throws IllegalStateException
     */
    public void terminate() {
        switch (state.get()) {
            case UNINITIALIZED:
                throw new IllegalStateException();
            case INITIALIZING:
                // INTENTIONAL FALL THROUGH
            case INITIALIZED:
                // INTENTIONAL FALL THROUGH
            case RESUMING:
                // INTENTIONAL FALL THROUGH
            case RESUMED:
                // INTENTIONAL FALL THROUGH
            case LOOPING:
                // INTENTIONAL FALL THROUGH
            case SUSPENDING:
                // INTENTIONAL FALL THROUGH
            case SUSPENDED: {
                doTerminate();
                break;
            }
            case DESTROYING:
                // INTENTIONAL FALL THROUGH
            case DESTROYED:
                // INTENTIONAL FALL THROUGH
            default:
                throw new IllegalStateException();
        }
    }

    /**
     * Get the current state of the plugin under execution
     */
    public PluginState getState() {
        return state.get();
    }
    /**
     * Enum used to describe the values of the plugin state
     */
    public enum PluginState {
        UNINITIALIZED, // Initial state of system
        INITIALIZING, INITIALIZED, RESUMING, RESUMED, LOOPING, SUSPENDING, SUSPENDED, DESTROYING, DESTROYED
    }

    /**
     * Generic task worker thread, synchronously pulls tasks off its queue and then executes them
     * asynchronously
     */
    protected class PluginWorker implements Runnable {
        protected BlockingQueue<Runnable> tasks;

        PluginWorker(BlockingQueue<Runnable> tasks) {
            this.tasks = tasks;
        }

        @Override public void run() {
            while (!Thread.currentThread().isInterrupted()) {
                try {
                    Runnable task = tasks.take();
                    task.run();
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt(); // Retrigger the interruption
                    break;
                }
            }
        }
    }
}
