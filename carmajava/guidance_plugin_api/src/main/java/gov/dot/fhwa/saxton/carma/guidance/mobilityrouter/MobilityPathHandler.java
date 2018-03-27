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

package gov.dot.fhwa.saxton.carma.guidance.mobilityrouter;

import cav_msgs.MobilityPath;
import gov.dot.fhwa.saxton.carma.guidance.conflictdetector.ConflictSpace;

/**
 * Callback interface for handling inbound relevant {@link MobilityPath} messages
 */
public interface MobilityPathHandler {
  /**
   * Callback to be invoked upon receipt of a relevant {@link MobilityPath} message which has a conflict
   * 
   * @param msg The MobilityPath message
   * @param hasConflict True/false if the path data contained in the MobilityPath message conflicts with our current projected path
   * @param conflictSpace The data related to the physical parameters of the conflict
   */
  void handleMobilityPathMessageWithConflict(MobilityPath msg, boolean hasConflict, ConflictSpace conflictSpace);
}