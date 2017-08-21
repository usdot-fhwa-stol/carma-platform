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

import cav_msgs.SystemAlert;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.IPublicationChannel;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.PubSubManager;

/**
 * Guidance package PluginManager component
 * <p>
 * Responsible for instantiating, running, owning, and runtime management of
 * all plugins installed in the software's operating environment.
 */
public class PluginManager implements Runnable {
  public PluginManager(PubSubManager pubSubManager) {
    this.pubSubManager = pubSubManager;
  }

  @Override public void run() {
    IPublicationChannel<SystemAlert> pub =
            pubSubManager.getPublicationChannelForTopic("system_alert", cav_msgs.SystemAlert._TYPE);
    for (; ; ) {
      cav_msgs.SystemAlert systemAlertMsg = pub.newMessage();
      systemAlertMsg.setDescription("Hello World! I am " + componentName + ". " + sequenceNumber++);
      systemAlertMsg.setType(cav_msgs.SystemAlert.SYSTEM_READY);
      pub.publish(systemAlertMsg);

      try {
        Thread.sleep(sleepDurationMillis);
      } catch (InterruptedException e) {
        // Ignore
      }
    }
  }

  // Member variables
  protected PubSubManager pubSubManager;
  protected int sequenceNumber = 0;
  protected final String componentName = "PluginManager";
  protected final long sleepDurationMillis = 1000;
}
