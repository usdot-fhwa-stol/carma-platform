/*
 * Copyright (C) 2018-2019 LEIDOS.
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

import cav_msgs.MobilityRequest;
import gov.dot.fhwa.saxton.carma.guidance.conflictdetector.ConflictSpace;

/**
 * Callback interface for handling inbound relevant {@link MobilityRequest} messages
 */
public interface MobilityRequestHandler {
  /**
   * Callback to be invoked upon receipt of a relevant {@link MobilityRequest} message
   * 
   * @param msg The MobilityRequest message
   * @param hasConflict True/false if the path data contained in the MobilityRequest message conflicts with our current projected path
   * @param conflictSpace The data related to the physical parameters of the conflict space
   * 
   * @return A MobilityRequestResponse enum value indicating whether the proposed trajectory is acceptable (ACK), unacceptable (NACK), or indifference (NO_RESPONSE)
   * By contract a response of ACK indicates that the responding plugin will handle any necessary adjustments to avoid collision and that NACK indicates no adjustments
   * will be made.
   */
  MobilityRequestResponse handleMobilityRequestMessage(MobilityRequest msg, boolean hasConflict, ConflictSpace conflictSpace);
}