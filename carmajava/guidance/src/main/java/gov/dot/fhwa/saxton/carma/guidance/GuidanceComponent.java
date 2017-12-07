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
import gov.dot.fhwa.saxton.carma.guidance.util.ILogger;
import gov.dot.fhwa.saxton.carma.guidance.util.LoggerManager;

import org.ros.concurrent.CancellableLoop;
import org.ros.node.ConnectedNode;

import cav_msgs.SystemAlert;

import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;

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

    public GuidanceComponent(GuidanceStateMachine stateMachine, IPubSubService pubSubService, ConnectedNode node) {
        this.stateMachine = stateMachine;
        this.node = node;
        this.pubSubService = pubSubService;
        this.log = LoggerManager.getLogger(this.getClass().getCanonicalName());
        this.jobQueue = new LinkedBlockingQueue<>();
    }

    /**
     * Get the human readable String representation of this component's name
     */
    public abstract String getComponentName();

    /**
     * Simple job-queue loop with jobs being added based on state machine transitions
     */
    public final void loop() throws InterruptedException {
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
        }
    }
    
    @Override
    public final void run() {
        Thread.currentThread().setName(getComponentName() + "Runner");
        log.info("STARTUP", getComponentName() + " is starting up.");
        CancellableLoop loop = new CancellableLoop() {
            @Override
            protected void loop() throws InterruptedException {
                GuidanceComponent.this.loop();
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

    /**
     * Job queue task for performing the shutting down process
     * 
     * Will log the fatal condition, alert the other ROS nodes in the CAV network to begin
     * shutdown procedures and then trigger GuidanceComponent activities to cease as well.
     */
    protected class Shutdown implements Runnable {

        String message = "";
        
        public Shutdown(String message) {
            this.message = message;
        }

        @Override
        public void run() {
            // Log the fatal error
            log.fatal("!!!!! Guidance component " + getComponentName() + " has entered a PANIC state !!!!!");
            log.fatal(message);
            
            // Alert the other ROS nodes to the FATAL condition
            IPublisher<SystemAlert> pub = pubSubService.getPublisherForTopic("system_alert", SystemAlert._TYPE);
            SystemAlert fatalBroadcast = pub.newMessage();
            fatalBroadcast.setDescription(getComponentName() + " has triggered a Guidance PANIC: " + message);
            fatalBroadcast.setType(SystemAlert.FATAL);
            pub.publish(fatalBroadcast);
            
            // Cancel the loop
            timingLoopThread.interrupt();
            loopThread.interrupt();
        }
    }

}
