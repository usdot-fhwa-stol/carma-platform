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

package gov.dot.fhwa.saxton.carma.lateralcontroldriver;

import org.ros.message.Time;

public interface ILateralControlDriver {

  /**
  * Sends an instructions message to the UI
  * @param msg - the message that is to be sent to notify the ui
  */
  void publishUIMessage(cav_msgs.UIInstructions msg);

  /**
   * Gets the current time
   *
   * @return The time
   */
  Time getTime();
}
