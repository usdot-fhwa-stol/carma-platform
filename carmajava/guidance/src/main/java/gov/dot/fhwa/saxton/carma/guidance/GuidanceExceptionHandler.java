package gov.dot.fhwa.saxton.carma.guidance;

import java.util.concurrent.atomic.*;
import org.apache.commons.logging.Log;

import cav_msgs.SystemAlert;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.*;

/**
 * Top level exception handling logic for Guidance.
 * </p>
 * Should be invoked where uncaught exceptions are involved in GuidanceComponent execution and in
 * the Guidance framework writ-large. Provides facilities to signal a system-wide PANIC state in
 * the event of unrecoverable conditions arising.
 */
public class GuidanceExceptionHandler {
    protected final Log log;
    protected AtomicReference<GuidanceState> state;
    protected IPubSubService pubSubService;
    protected IPublisher<SystemAlert> systemAlertPub;

    GuidanceExceptionHandler(AtomicReference<GuidanceState> state, Log log) {
        this.state = state;
        this.log = log;
    }

    /**
     * Initialize the GuidanceExceptionHandler prior to use
     * </p>
     * Connect to the SystemAlert topic such that Guidance panics may be broadcast to the greater ROS network.
     * Initializing this way works around an awkward circular dependency between IPubSubService and
     * GuidanceExceptionHandler
     */
    public void init(IPubSubService pubSubService) {
        systemAlertPub = pubSubService.getPublisherForTopic("system_alert", SystemAlert._TYPE);
    }

    /**
     * Handle an exception that hasn't been caught anywhere else
     * </p>
     * Will result in guidance shutdown and publishing of SystemAlert.FATAL
     */
    public void handleException(Throwable e) {
        // This code feels redundant with GuidanceComponent#panic() but I'm not sure how to handle that
        // GuidanceComponent#panic() is an intentional recognition that the scenario is beyond our control
        // And this is that some unexpected uncontrollable event has occurred. I like differentiating b/w
        // these cases but it leads to redundant code like this. Maybe a refactor is in order?
        log.error("Global guidance exception handler caught exception from thread: " + Thread.currentThread().getName(),
                e);

        state.set(GuidanceState.SHUTDOWN);

        if (systemAlertPub != null) {
            SystemAlert alert = systemAlertPub.newMessage();
            alert.setDescription("Guidance panic triggered in thread " + Thread.currentThread().getName()
                    + " by an uncaught exception!");
            alert.setType(SystemAlert.FATAL);
            systemAlertPub.publish(alert);
        } else {
            log.fatal("Exception raised before GuidanceExceptionHandler fully initialized!!!"
                    + "Unable to publish SystemAlert.FATAL. System may be in undefined state!!!");
        }
    }
}