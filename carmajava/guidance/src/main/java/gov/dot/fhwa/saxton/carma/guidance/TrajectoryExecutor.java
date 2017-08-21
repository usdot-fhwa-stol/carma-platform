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
import gov.dot.fhwa.saxton.carma.guidance.pubsub.IPublicationChannel;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.PubSubManager;

/**
 * Guidance package TrajectoryExecutor component
 * <p>
 * Guidance component responsible for performing the timely execution of planned
 * maneuvers in a Trajectory planned by the Arbitrator and the Guidance package's
 * currently configured plugins.
 */
public class TrajectoryExecutor implements Runnable {
  public TrajectoryExecutor(PubSubManager pubSubManager) {
    this.pubSubManager = pubSubManager;
  }

  @Override public void run() {
    IPublicationChannel<SystemAlert> pub =
            pubSubManager.getPublicationChannelForTopic("system_alert", cav_msgs.SystemAlert._TYPE);
    for (; ; ) {
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
  protected final String componentName = "TrajectoryExecutor";
  protected PubSubManager pubSubManager;
  protected int sequenceNumber = 0;
  protected final long sleepDurationMillis = 30000;
}
