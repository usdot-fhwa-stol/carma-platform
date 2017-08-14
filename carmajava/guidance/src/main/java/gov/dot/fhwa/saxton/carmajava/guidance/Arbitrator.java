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

//TODO: Naming convention of "package gov.dot.fhwa.saxton.carmajava.<template>;"
//Originally "com.github.rosjava.carmajava.template;"
package gov.dot.fhwa.saxton.carmajava.guidance;

import org.apache.commons.logging.Log;
import org.ros.message.MessageListener;
import org.ros.node.topic.Subscriber;

import org.ros.concurrent.CancellableLoop;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;

import org.ros.node.parameter.ParameterTree;
import org.ros.namespace.NameResolver;
import org.ros.message.MessageFactory;

/**
 * Guidance package Arbitrator component
 *
 * Runs inside the GuidanceMain class's executor and prompts the Guidance.Plugins
 * to plan trajectories for the vehicle to execute.
 */
public class Arbitrator implements Runnable {
    public Arbitrator(PubSubManager pubSubManager) {
        this.pubSubManager = pubSubManager;
    }

    @Override
    public void run() {
        for (;;) {
            pubSubManager.publish("Hello World! I am"
                                  + componentName
                                  +  "."
                                  + sequenceNumber++);

            try {
                Thread.sleep(sleepDurationMillis);
            } catch (InterruptedException e) {
                // Ignore
            }
        }
    }

    // Member variables
    protected PubSubManager pubSubManager;
    protected final String componentName = "Arbitrator";
    protected int sequenceNumber = 0;
    protected final long sleepDurationMillis = 1000;
}
