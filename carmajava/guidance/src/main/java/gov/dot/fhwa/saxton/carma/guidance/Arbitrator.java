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

import gov.dot.fhwa.saxton.carma.guidance.pubsub.PubSubManager;

/**
 * Guidance package Arbitrator component
 * <p>
 * Runs inside the GuidanceMain class's executor and prompts the Guidance.Plugins
 * to plan trajectories for the vehicle to execute.
 */
public class Arbitrator implements Runnable {
  public Arbitrator(PubSubManager pubSubManager) {
    this.pubSubManager = pubSubManager;
  }

  @Override public void run() {
    for (; ; ) {
      pubSubManager.publish("Hello World! I am " + componentName + ". " + sequenceNumber++);

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
