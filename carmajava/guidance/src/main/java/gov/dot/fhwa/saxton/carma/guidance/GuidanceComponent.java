/*
 * Copyright (C) 2018 LEIDOS.
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
import gov.dot.fhwa.saxton.carma.guidance.util.ILogger;
import gov.dot.fhwa.saxton.carma.guidance.util.LoggerManager;

import org.ros.concurrent.CancellableLoop;
import org.ros.node.ConnectedNode;

import cav_msgs.SystemAlert;

import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.atomic.AtomicReference;

/**
 * Base class for all Guidance components.
 * <p>
 * Defines the execution framework within the context of both Guidance and the overall system's state
 * due to driver initialization and user command.
 */
public abstract class GuidanceComponent implements Runnable {
    
    protected final long DEFAULT_LOOP_SLEEP_MS = 10000;
    
    protected ConnectedNode node;
    protected IPubSubService pubSubService;
    protected ILogger log;
    protected BlockingQueue<Runnable> jobQueue;
    protected GuidanceStateMachine stateMachine;
    protected Thread loopThread;
    protected Thread timingLoopThread;
    protected AtomicReference<GuidanceState> currentState;
    protected GuidanceExceptionHandler exceptionHandler;
    

    public GuidanceComponent(GuidanceStateMachine stateMachine, IPubSubService pubSubService, ConnectedNode node) {
        // In GuidanceComponent, we only use stateMachine for get current state and process PANIC event
        this.stateMachine = stateMachine;
        this.node = node;
        this.pubSubService = pubSubService;
        this.log = LoggerManager.getLogger(this.getClass().getCanonicalName());
        this.jobQueue = new LinkedBlockingQueue<>();
        this.currentState = new AtomicReference<GuidanceState>(GuidanceState.STARTUP);
        this.exceptionHandler = new GuidanceExceptionHandler(stateMachine);
    }

    /**
     * Get the human readable String representation of this component's name
     */
    public abstract String getComponentName();

    /**
     * Get called once a component is initialized
     */
    public abstract void onStartup();
    
    /**
     * Get called once a component is received a system ready alert
     */
    public abstract void onSystemReady();
    
    /**
     * Get called once route is selected and active
     */
    public abstract void onRouteActive();
    
    /**
     * Get called once vehicle ACC is engaged
     */
    public abstract void onEngaged();
    
    /**
     * Get called once guidance component received a request to restart
     */
    public abstract void onCleanRestart();
    
    /**
     * Get called once guidance component found controller timeout when guidance is engaged
     */
    public abstract void onDeactivate();
    
    /**
     * Job queue task for performing the shutting down process
     * Will log the fatal condition, alert the other ROS nodes in the CAV network to begin
     * shutdown procedures and then trigger GuidanceComponent activities to cease as well.
     */
    public void onShutdown() {
        currentState.set(GuidanceState.SHUTDOWN);

        log.info(getComponentName() + " shutting down normally.");
        
        // Cancel the loop
        timingLoopThread.interrupt();
        loopThread.interrupt();
    }

    /**
     * Generic handler for panic conditions, just immediately shutdown and log this
     */
    public void onPanic() {
        currentState.set(GuidanceState.SHUTDOWN);
        
        // Log the fatal error
        log.fatal(getComponentName() + " has activated panic procedures. Shutting down immediately.");
        
        // Cancel the loop
        timingLoopThread.interrupt();
        loopThread.interrupt();
    }
    
    /**
     * Simple job-queue loop with jobs being added based on state machine transitions
     */
    public final void runJobQueue() throws InterruptedException {
        log.debug(this.getComponentName() + " is polling job queue.");
        Runnable job = jobQueue.take();
        log.debug(this.getComponentName() + " found job of type: " + job.getClass().getSimpleName() + ". Executing!");
        job.run();
    }

    public void timingLoop() throws InterruptedException {
        try {
            Thread.sleep(DEFAULT_LOOP_SLEEP_MS);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
            throw e;
        }
    }
    
    @Override
    public final void run() {
        Thread.currentThread().setName(getComponentName() + "Runner");
        log.info("STARTUP", getComponentName() + " is starting up.");
        CancellableLoop loop = new CancellableLoop() {
            @Override
            protected void loop() throws InterruptedException {
                GuidanceComponent.this.runJobQueue();
            }
        };
        CancellableLoop timingLoop = new CancellableLoop() {
            @Override
            protected void loop() throws InterruptedException {
                GuidanceComponent.this.timingLoop();
            }
        };
        loopThread = new Thread(loop);
        loopThread.setName(getComponentName() + "Looper");
        timingLoopThread = new Thread(timingLoop);
        timingLoopThread.setName(getComponentName() + "TimingLooper");
        loopThread.start();
        timingLoopThread.start();
    }
}
