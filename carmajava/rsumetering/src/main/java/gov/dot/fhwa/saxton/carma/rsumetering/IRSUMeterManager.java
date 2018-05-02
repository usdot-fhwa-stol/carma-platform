/*
 * Copyright (C) 2018 LEIDOS.
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

package gov.dot.fhwa.saxton.carma.rsumetering;

import org.ros.message.Time;

import cav_msgs.MobilityOperation;
import cav_msgs.MobilityRequest;

public interface IRSUMeterManager {

  /**
  * Sends a MobilityRequest message for broadcast
  * @param msg - the message that is to be ssent
  */
  void publishMobilityRequest(MobilityRequest msg);

  /**
  * Sends a MobilityOperation message for broadcast
  * @param msg - the message that is to be ssent
  */
  void publishMobilityOperation(MobilityOperation msg);

  /**
   * Gets the current time
   *
   * @return The time
   */
  Time getTime();
}
