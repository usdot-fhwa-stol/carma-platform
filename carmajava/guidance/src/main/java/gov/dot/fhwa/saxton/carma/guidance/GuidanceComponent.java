/*
 * Copyright (C) 2017 LEIDOS.
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

package gov.dot.fhwa.saxton.carma.guidance;

import gov.dot.fhwa.saxton.carma.guidance.pubsub.*;
import org.apache.commons.logging.Log;
import org.ros.concurrent.CancellableLoop;
import org.ros.node.ConnectedNode;

import cav_msgs.SystemAlert;

import java.util.concurrent.atomic.AtomicReference;

/**
 * Base class for all Guidance components.
 * <p>
 * Defines the execution framework within the context of both Guidance and the overall system's state
 * due to driver initialization and user command.
 */
public abstract class GuidanceComponent implements Runnable {
    protected ConnectedNode node;
    protected IPubSubService pubSubService;
    protected Log log;
    protected Thread loopThread;

    private final long WAIT_DURATION_MS = 200;
    private final long DEFAULT_LOOP_SLEEP_MS = 5000;

    private AtomicReference<GuidanceState> state;

    public GuidanceComponent(AtomicReference<GuidanceState> state, IPubSubService pubSubService, ConnectedNode node) {
        this.state = state;
        this.node = node;
        this.pubSubService = pubSubService;
        this.log = node.getLog();
    }

    /**
     * Get the human readable String representation of this component's name
     */
    public abstract String getComponentName();

    /**
     * An event callback called once when Guidance is first started up. Do not loop in this handler.
     */
    public abstract void onGuidanceStartup();

    /**
     * An event callback called once when the CAV platform drivers are initialized. Do not loop in
     * this handler.
     */
    public abstract void onSystemReady();

    /**
     * An event callback called once when the user activates the Guidance module via the user
     * interface. Do not loop in this handler.
     */
    public abstract void onGuidanceEnable();

    /**
     * Primary execution loop for this Guidance component. Execution will be interrupted when a state
     * change occurs. This method is called within a tight while loop which will not exit until
     * interrupted. This method will be called after each event handler has ended. Timing of this
     * loop is the sub-classers responsibility.
     *
     * Default behavior if not overridden is to sleep for 5s.
     */
    public void loop() throws InterruptedException {
        try {
            Thread.sleep(DEFAULT_LOOP_SLEEP_MS);
        } catch (InterruptedException e) {
        }
    }

    public final void run() {
        Thread.currentThread().setName(getComponentName() + "Runner");
        log.info(getComponentName() + " starting up.");
        onGuidanceStartup();
        CancellableLoop loop = new CancellableLoop() {
            @Override
            protected void loop() throws InterruptedException {
                GuidanceComponent.this.loop();
            }
        };
        loopThread = new Thread(loop);
        loopThread.setName(getComponentName() + "Looper");
        loopThread.start();

        // Wait for DRIVERS_READY
        while (state.get() == GuidanceState.STARTUP) {
            try {
                Thread.sleep(WAIT_DURATION_MS);
            } catch (InterruptedException e) {
            }
        }
        log.info(getComponentName() + " transitioning to DRIVERS_READY state.");
        cancelAndWaitForLoop(loop);

        onSystemReady();
        loop = new CancellableLoop() {
            @Override
            protected void loop() throws InterruptedException {
                GuidanceComponent.this.loop();
            }
        };
        loopThread = new Thread(loop);
        loopThread.setName(getComponentName() + "Looper");
        loopThread.start();

        // Wait for GUIDANCE_ENGAGED
        while (!(state.get() == GuidanceState.ENGAGED)) {
            try {
                Thread.sleep(WAIT_DURATION_MS);
            } catch (InterruptedException e) {
            }
        }
        cancelAndWaitForLoop(loop);

        log.info(getComponentName() + " transitioning to ENGAGED state.");
        onGuidanceEnable();
        loop = new CancellableLoop() {
            @Override
            protected void loop() throws InterruptedException {
                GuidanceComponent.this.loop();
            }
        };
        loopThread = new Thread(loop);
        loopThread.setName(getComponentName() + "Looper");
        loopThread.start();

        // Wait for Guidance to shutdown
        while (!(state.get() == GuidanceState.SHUTDOWN)) {
            try {
                Thread.sleep(WAIT_DURATION_MS);
            } catch (InterruptedException e) {
            }
        }
        cancelAndWaitForLoop(loop);

        log.info(getComponentName() + " shutting down.");
        onGuidanceShutdown();
        log.info(getComponentName() + " shut down.");
    }

    /**
     * Optionally overridable method to hook when the Guidance node is gracefuly shutting down
     * or has received a SystemAlert.FATAL message
     *
     * Default implementation does nothing.
     */
    public void onGuidanceShutdown() {
        // NO-OP
    }

    /**
     * Get the Guidance component's current state value
     */
    protected final GuidanceState getState() {
        return state.get();
    }

    /**
     * Cancel the loop and sleep until it actually exits. Functionally kill it and then join
     * @param loop The loop to be canceled
     */
    private void cancelAndWaitForLoop(CancellableLoop loop) {
        loop.cancel();
        while (loop.isRunning()) {
            try {
                Thread.sleep(WAIT_DURATION_MS);
            } catch (InterruptedException e) {
            }
        }
    }

    /**
     * Signal a panic in a Guidance component that necessitates full system shutdown
     * 
     * Will log the fatal condition, alert the other ROS nodes in the CAV network to begin
     * shutdown procedures and then trigger all other Guidance activity to cease as well.
     */
    protected final void panic(String message) {
        // Log the fatal error
        log.fatal("!!!!! Guidance component " + getComponentName()
                + " has entered a PANIC state !!!!!\nCause of panic: " + message);
        log.fatal("Publishing cav_msgs/SystemAlert.FATAL");

        // Alert the other ROS nodes to the FATAL condition
        IPublisher<SystemAlert> pub = pubSubService.getPublisherForTopic("system_alert", SystemAlert._TYPE);
        SystemAlert fatalBroadcast = pub.newMessage();
        fatalBroadcast.setDescription(getComponentName() + " has triggered a Guidance PANIC: " + message);
        fatalBroadcast.setType(SystemAlert.FATAL);
        pub.publish(fatalBroadcast);

        // Signal the other guidance components to shutdown as well
        state.set(GuidanceState.SHUTDOWN);
    }

}
