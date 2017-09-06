/*
 * TODO: Copyright (C) 2017 LEIDOS.
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

import cav_msgs.RouteState;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.IPubSubService;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.ISubscriber;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.OnMessageCallback;
import org.apache.commons.logging.Log;
import org.ros.node.ConnectedNode;

/**
 * Guidance package Arbitrator component
 * <p>
 * Runs inside the GuidanceMain class's executor and prompts the Guidance.Plugins
 * to plan trajectories for the vehicle to execute.
 */
public class Arbitrator implements Runnable {
    protected final long sleepDurationMillis = 30000;
    protected IPubSubService iPubSubService;
    protected ConnectedNode node;
    protected Log log;

    Arbitrator(IPubSubService iPubSubService, ConnectedNode node) {
        this.iPubSubService = iPubSubService;
        this.node = node;
        this.log = node.getLog();
    }

    @Override public void run() {
        log.info("Arbitrator running!");
        ISubscriber<RouteState> routeStateSubscriber = iPubSubService.getSubscriberForTopic("route_status", RouteState._TYPE);
        routeStateSubscriber.registerOnMessageCallback(new OnMessageCallback<RouteState>() {
            @Override public void onMessage(RouteState msg) {
                log.info("Received RouteState:" + msg);
            }
        });

        for (; ; ) {
            try {
                Thread.sleep(sleepDurationMillis);
            } catch (InterruptedException e) {
                // Ignore
            }
        }
    }
}
