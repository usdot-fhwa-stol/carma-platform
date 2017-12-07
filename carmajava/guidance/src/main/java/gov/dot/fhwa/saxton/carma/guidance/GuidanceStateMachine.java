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

import java.util.List;
import java.util.Vector;
import java.util.concurrent.atomic.AtomicReference;

import gov.dot.fhwa.saxton.carma.guidance.util.ILogger;
import gov.dot.fhwa.saxton.carma.guidance.util.LoggerManager;

/**
 * This class is responsible for the logic of Guidance state transitions
 */
public class GuidanceStateMachine {
    private AtomicReference<GuidanceState> guidance_state = new AtomicReference<>(GuidanceState.STARTUP);
    private List<IStateChangeListener> listeners = new Vector<>();
    private ILogger log = LoggerManager.getLogger();

    /**
     * Define all state transitions in GuidanceStateMachine based on GuidanceEvent 
     * @param guidance_event
     */
    public synchronized void processEvent(GuidanceEvent guidance_event) {
        log.debug("Guidance state machine reveiced " + guidance_event + " at state: " + guidance_state.get());
        GuidanceState old_state = guidance_state.get();
        switch (old_state) {
        case STARTUP:
            if(guidance_event == GuidanceEvent.FOUND_DRIVERS) {
                guidance_state.set(GuidanceState.DRIVERS_READY);
            } else if(guidance_event == GuidanceEvent.PANIC) {
                guidance_state.set(GuidanceState.SHUTDOWN);
            }
            break;
        case DRIVERS_READY:
            if(guidance_event == GuidanceEvent.ACTIVATE_ROUTE) {
                guidance_state.set(GuidanceState.ACTIVE);
            } else if(guidance_event == GuidanceEvent.PANIC) {
                guidance_state.set(GuidanceState.SHUTDOWN);
            }
            break;
        case ACTIVE:
            if(guidance_event == GuidanceEvent.START_ROUTE) {
                guidance_state.set(GuidanceState.ENGAGED);
            } else if(guidance_event == GuidanceEvent.LEFT_ROUTE) {
                guidance_state.set(GuidanceState.DRIVERS_READY);
            } else if(guidance_event == GuidanceEvent.PANIC) {
                guidance_state.set(GuidanceState.SHUTDOWN);
            }
            break;
        case ENGAGED:
            if(guidance_event == GuidanceEvent.FINISH_ROUTE) {
                guidance_state.set(GuidanceState.DRIVERS_READY);
            } else if(guidance_event == GuidanceEvent.LEFT_ROUTE) {
                guidance_state.set(GuidanceState.DRIVERS_READY);
            } else if(guidance_event == GuidanceEvent.PANIC) {
                guidance_state.set(GuidanceState.SHUTDOWN);
            }
            break;
        default:
            break;
        }
        
        // Call all the listeners if we've changed state
        GuidanceState current_state = guidance_state.get(); 
        if(old_state != current_state) {
            log.debug("Guidance transitioned to state" + current_state);
            for(IStateChangeListener listener : listeners) {
                listener.onStateChange();
            }
        } else {
            log.debug("Guidance did not change state");
        }
    }
    
    /**
     * Get the current state of the guidance state machine
     * @return the current state of guidance
     */
    public GuidanceState getState() {
        return guidance_state.get();
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
