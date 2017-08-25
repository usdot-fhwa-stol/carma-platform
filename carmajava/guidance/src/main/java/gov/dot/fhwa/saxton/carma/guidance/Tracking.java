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
package gov.dot.fhwa.saxton.carma.guidance;

import cav_msgs.SystemAlert;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.IPubSubService;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.IPublicationChannel;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.IPublisher;
import org.apache.commons.logging.Log;

/**
 * Guidance package Tracking component
 * <p>
 * Reponsible for detecting when the vehicle strays from it's intended route or
 * trajectory and signalling the failure on the /system_alert topic
 */
public class Tracking implements Runnable {
  public Tracking(IPubSubService IPubSubService, Log log) {
    this.IPubSubService = IPubSubService;
    this.log = log;
  }

  @Override public void run() {
    IPublisher<SystemAlert> pub =
            IPubSubService.getPublisherForTopic("system_alert", cav_msgs.SystemAlert._TYPE);
    for (;;) {
      cav_msgs.SystemAlert systemAlertMsg = pub.newMessage();
      systemAlertMsg.setDescription("Hello World! I am " + componentName + ". " + sequenceNumber++);
      systemAlertMsg.setType(SystemAlert.CAUTION);
      pub.publish(systemAlertMsg);

      try {
        Thread.sleep(sleepDurationMillis);
      } catch (InterruptedException e) {
      }
    }
  }

  // Member variables
  protected IPubSubService IPubSubService;
  protected final String componentName = "Tracking";
  protected int sequenceNumber = 0;
  protected final long sleepDurationMillis = 30000;
  protected Log log;
}
