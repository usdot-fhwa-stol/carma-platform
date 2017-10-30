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

package gov.dot.fhwa.saxton.carma.guidance.plugins;

/**
 * Asynchronous task for calling a plug-in's loop function repeatedly until the thread is
 * interrupted.
 */
public class LoopPluginTask implements Runnable {
    protected TaskCompletionCallback callback;
    protected IPlugin plugin;

    LoopPluginTask(IPlugin plugin, TaskCompletionCallback callback) {
        this.plugin = plugin;
        this.callback = callback;
    }

    @Override public void run() {
        // Invoke the complete callback early since there is no LOOPED state only LOOPING
        callback.onComplete();
        while (!Thread.currentThread().isInterrupted()) {
            try {
                plugin.loop();
            } catch (InterruptedException e) {
                // Rethrow the interruption
                Thread.currentThread().interrupt();
            }
        }
    }
}
