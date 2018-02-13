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

import java.util.ArrayList;
import java.util.List;
import java.util.Vector;
import java.util.concurrent.atomic.AtomicReference;

import org.ros.concurrent.CancellableLoop;

import gov.dot.fhwa.saxton.carma.guidance.pubsub.IPubSubService;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.IPublisher;
import gov.dot.fhwa.saxton.carma.guidance.util.ILogger;
import gov.dot.fhwa.saxton.carma.guidance.util.LoggerManager;

/**
 * This class is responsible for the logic of Guidance state transitions
 */
public class GuidanceStateMachine {
    
    private static long LOOP_SLEEP_MS = 1000;
    
    private AtomicReference<GuidanceState> guidance_state = new AtomicReference<>(GuidanceState.STARTUP);
    private List<IStateChangeListener> listeners = new Vector<>();
    private ILogger log = LoggerManager.getLogger();
    private IPubSubService pubSubService;
    private IPublisher<cav_msgs.GuidanceState> statePub;
    private IPublisher<cav_msgs.GuidanceAction> actionPub;

    /**
     * Define all state transitions in GuidanceStateMachine based on GuidanceEvent 
     * @param guidance_event
     */
    public synchronized void processEvent(GuidanceEvent guidance_event) {
        log.debug("GUIDANCE_STATE", "Guidance state machine reveiced " + guidance_event + " at state: " + guidance_state.get());
        GuidanceState old_state = guidance_state.get();
        GuidanceAction action = null;
        if(guidance_event == GuidanceEvent.PANIC) {
            guidance_state.set(GuidanceState.SHUTDOWN);
            action = GuidanceAction.PANIC_SHUTDOWN;
        } else if(guidance_event == GuidanceEvent.SHUTDOWN) {
            guidance_state.set(GuidanceState.SHUTDOWN);
            action = GuidanceAction.SHUTDOWN;
        } else {
            switch (old_state) {
            case STARTUP:
                if(guidance_event == GuidanceEvent.FOUND_DRIVERS) {
                    guidance_state.set(GuidanceState.DRIVERS_READY);
                    action = GuidanceAction.INTIALIZE;
                }
                break;
            case DRIVERS_READY:
                if(guidance_event == GuidanceEvent.ACTIVATE) {
                    guidance_state.set(GuidanceState.ACTIVE);
                    action = GuidanceAction.ACTIVATE;
                }
                break;
            case ACTIVE:
                if(guidance_event == GuidanceEvent.START_ROUTE) {
                    guidance_state.set(GuidanceState.ENGAGED);
                    action = GuidanceAction.ENGAGE;
                } else if(guidance_event == GuidanceEvent.LEFT_ROUTE ||
                          guidance_event == GuidanceEvent.DISENGAGE ||
                          guidance_event == GuidanceEvent.FINISH_ROUTE) {
                    guidance_state.set(GuidanceState.DRIVERS_READY);
                    action = GuidanceAction.RESTART;
                }
                break;
            case INACTIVE:
                if(guidance_event == GuidanceEvent.START_ROUTE) {
                    guidance_state.set(GuidanceState.ENGAGED);
                    action = GuidanceAction.ENGAGE;
                } else if(guidance_event == GuidanceEvent.LEFT_ROUTE ||
                          guidance_event == GuidanceEvent.DISENGAGE ||
                          guidance_event == GuidanceEvent.FINISH_ROUTE) {
                    guidance_state.set(GuidanceState.DRIVERS_READY);
                    action = GuidanceAction.RESTART;
                }
                break;
            case ENGAGED:
                if(guidance_event == GuidanceEvent.ROBOT_DISABLED) {
                    guidance_state.set(GuidanceState.INACTIVE);
                    action = GuidanceAction.DEACTIVATE;
                } else if(guidance_event == GuidanceEvent.LEFT_ROUTE ||
                          guidance_event == GuidanceEvent.DISENGAGE ||
                          guidance_event == GuidanceEvent.FINISH_ROUTE) {
                    guidance_state.set(GuidanceState.DRIVERS_READY);
                    action = GuidanceAction.RESTART;
                }
                break;
            default:
                log.warn("GUIDANCE_STATE", "Guidance state machine take NO action on event " + guidance_event + " at state " + old_state);
            }
        }     
        
        // Call all the listeners if we've changed state
        GuidanceState current_state = guidance_state.get(); 
        if(old_state != current_state) {
            log.debug("GUIDANCE_STATE", "Guidance transited to state " + current_state);
            if(action != null) {
                cav_msgs.GuidanceAction actionMsg = actionPub.newMessage();
                log.debug("GUIDANCE_STATE", "Guidance is taking action " + action.name());
                switch (action) {
                case ACTIVATE:
                    actionMsg.setAction(cav_msgs.GuidanceAction.ACTIVATE);
                    break;
                case DEACTIVATE:
                    actionMsg.setAction(cav_msgs.GuidanceAction.DEACTIVATE);
                    break;
                case ENGAGE:
                    actionMsg.setAction(cav_msgs.GuidanceAction.ENGAGE);
                    break;
                case INTIALIZE:
                    actionMsg.setAction(cav_msgs.GuidanceAction.INTIALIZE);
                    break;
                case RESTART:
                    actionMsg.setAction(cav_msgs.GuidanceAction.RESTART);
                    break;
                case PANIC_SHUTDOWN:
                    actionMsg.setAction(cav_msgs.GuidanceAction.PANIC_SHUTDOWN);
                    break;
                case SHUTDOWN:
                    actionMsg.setAction(cav_msgs.GuidanceAction.SHUTDOWN);
                    break;
                default:
                    break;
                }
                List<IStateChangeListener> tmpListener = new ArrayList<IStateChangeListener>();
                synchronized(listeners) {
                    tmpListener.addAll(listeners);
                }
                for(IStateChangeListener listener : tmpListener) {
                    listener.onStateChange(action);
                }
                actionPub.publish(actionMsg);
            }
        } else {
            log.debug("GUIDANCE_STATE", "Guidance did not change state");
        }
    }
    
    /**
     * Get the current state of the guidance state machine
     * @return the current state of guidance
     */
    public GuidanceState getState() {
        return guidance_state.get();
    }
    
    public void initSubPub(IPubSubService pubSubService) {
        this.pubSubService = pubSubService;
        statePub = this.pubSubService.getPublisherForTopic("state", cav_msgs.GuidanceState._TYPE);   
        actionPub = this.pubSubService.getPublisherForTopic("action", cav_msgs.GuidanceAction._TYPE);
        CancellableLoop pubLoop = new CancellableLoop() {
            @Override
            protected void loop() throws InterruptedException {
                GuidanceStateMachine.this.loop();
            }
        };
        Thread pubThread = new Thread(pubLoop);
        pubThread.start();
    }
    
    public void loop() throws InterruptedException {
        cav_msgs.GuidanceState state = statePub.newMessage();
        switch (guidance_state.get()) {
        case ACTIVE:
            state.setState(cav_msgs.GuidanceState.ACTIVE);
            break;
        case INACTIVE:
            state.setState(cav_msgs.GuidanceState.INACTIVE);
            break;
        case DRIVERS_READY:
            state.setState(cav_msgs.GuidanceState.DRIVERS_READY);
            break;
        case ENGAGED:
            state.setState(cav_msgs.GuidanceState.ENGAGED);
            break;
        case SHUTDOWN:
            state.setState(cav_msgs.GuidanceState.SHUTDOWN);
            break;
        case STARTUP:
            state.setState(cav_msgs.GuidanceState.STARTUP);
            break;
        default:
            break;
        }
        statePub.publish(state);
        try {
            Thread.sleep(LOOP_SLEEP_MS);
        } catch(InterruptedException e) {
            Thread.currentThread().interrupt();
            throw e;
        }
    }
    
    /**
     * Register a state change listener to be called when the state changes
     * @param newListener
     */
    public void registerStateChangeListener(IStateChangeListener newListener) {
        listeners.add(newListener);
    }
    
    /**
     * Unregister a state change listener
     */
    public void unregisterStateChangeListener(IStateChangeListener newListener) {
        listeners.remove(newListener);
    }
}
