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

import gov.dot.fhwa.saxton.carma.guidance.util.ILogger;

/**
 * Top level exception handling logic for Guidance.
 * </p>
 * Should be invoked where uncaught exceptions are involved in GuidanceComponent execution and in
 * the Guidance framework writ-large. Provides facilities to signal a system-wide PANIC event in
 * the event of unrecoverable conditions arising.
 */
public class GuidanceExceptionHandler {
    
    private ILogger log;
    private GuidanceStateMachine stateMachine;
    
    public GuidanceExceptionHandler(ILogger log, GuidanceStateMachine stateMachine) {
        this.log = log;
        this.stateMachine = stateMachine;
    }

    /**
     * Handle an exception that hasn't been caught anywhere else.
     * Will result in guidance shutdown.
     */
    public void handleException(Throwable e) {
        log.error("Global guidance exception handler caught exception from thread: " + Thread.currentThread().getName(), e);
        stateMachine.processEvent(GuidanceEvent.PANIC);
    }

}