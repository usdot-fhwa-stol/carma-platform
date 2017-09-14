package gov.dot.fhwa.saxton.carma.guidance;

import cav_msgs.SystemAlert;
import cav_srvs.SetGuidanceEnabled;
import cav_srvs.SetGuidanceEnabledRequest;
import cav_srvs.SetGuidanceEnabledResponse;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.*;
import org.apache.commons.logging.Log;
import org.ros.concurrent.CancellableLoop;
import org.ros.node.ConnectedNode;

import java.util.concurrent.atomic.AtomicReference;

public abstract class GuidanceComponent implements Runnable {
    protected ConnectedNode node;
    protected IPubSubService pubSubService;
    protected Log log;

    private final long WAIT_DURATION_MS = 200;

    private ISubscriber<SystemAlert> systemAlertSubscriber;
    private IService<SetGuidanceEnabledRequest, SetGuidanceEnabledResponse> guidanceEnabledSubscriber;
    private AtomicReference<GuidanceState> state;

    public GuidanceComponent(AtomicReference<GuidanceState> state, IPubSubService pubSubService, ConnectedNode node) {
        this.state = state;
        this.node = node;
        this.pubSubService = pubSubService;
        this.log = node.getLog();
    }

    public abstract String getComponentName();
    public abstract void onGuidanceStartup();
    public abstract void onSystemReady();
    public abstract void onGuidanceEnable();
    public abstract void loop();

    public final void run() {
        log.info(getComponentName() + " starting up.");
        onGuidanceStartup();
        CancellableLoop loop = new CancellableLoop() {
            @Override protected void loop() throws InterruptedException {
                GuidanceComponent.this.loop();
            }
        };
        loop.run();

        // Wait for DRIVERS_READY
        while (state.get() == GuidanceState.STARTUP) {
            try {
                Thread.sleep(WAIT_DURATION_MS);
            } catch (InterruptedException e) {
            }
        }
        cancelAndWaitForLoop(loop);

        log.info(getComponentName() + " transitioning to DRIVERS_READY state.");
        onSystemReady();
        loop = new CancellableLoop() {
            @Override protected void loop() throws InterruptedException {
                GuidanceComponent.this.loop();
            }
        };
        cancelAndWaitForLoop(loop);

        // Wait for GUIDANCE_ENABLE
        while (!(state.get() == GuidanceState.ENABLED)) {
            try {
                Thread.sleep(WAIT_DURATION_MS);
            } catch (InterruptedException e) {
            }
        }

        log.info(getComponentName() + " transitioning to ENABLED state.");
        onGuidanceEnable();
        loop = new CancellableLoop() {
            @Override protected void loop() throws InterruptedException {
                GuidanceComponent.this.loop();
            }
        };
        loop.run();
    }

    protected GuidanceState getState() {
        return state.get();
    }

    private void cancelAndWaitForLoop(CancellableLoop loop) {
        loop.cancel();
        while (loop.isRunning()) {
            try {
                Thread.sleep(WAIT_DURATION_MS);
            } catch (InterruptedException e) {
            }
        }
    }

}
