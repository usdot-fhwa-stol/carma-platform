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

import java.util.concurrent.atomic.*;
import gov.dot.fhwa.saxton.carma.guidance.util.ILogger;
import gov.dot.fhwa.saxton.carma.guidance.util.LoggerManager;

/**
 * Top level exception handling logic for Guidance.
 * </p>
 * Should be invoked where uncaught exceptions are involved in GuidanceComponent execution and in
 * the Guidance framework writ-large. Provides facilities to signal a system-wide PANIC state in
 * the event of unrecoverable conditions arising.
 */
public class GuidanceExceptionHandler {
    protected final ILogger log = LoggerManager.getLogger();
    protected AtomicReference<GuidanceState> state;

    GuidanceExceptionHandler(AtomicReference<GuidanceState> state) {
        this.state = state;
    }

    /**
     * Handle an exception that hasn't been caught anywhere else.
     * Will result in guidance shutdown.
     */
    public void handleException(Throwable e) {
        // This code feels redundant with GuidanceComponent#panic() but I'm not sure how to handle that
        // GuidanceComponent#panic() is an intentional recognition that the scenario is beyond our control
        // And this is that some unexpected uncontrollable event has occurred. I like differentiating b/w
        // these cases but it leads to redundant code like this. Maybe a refactor is in order?
        log.error("Global guidance exception handler caught exception from thread: " + Thread.currentThread().getName(),
                e);

        state.set(GuidanceState.SHUTDOWN);
    }

}