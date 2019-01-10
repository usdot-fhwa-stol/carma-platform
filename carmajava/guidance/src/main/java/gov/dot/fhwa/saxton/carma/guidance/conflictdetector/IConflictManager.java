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

package gov.dot.fhwa.saxton.carma.guidance.conflictdetector;

import java.util.List;

import gov.dot.fhwa.saxton.carma.guidance.util.trajectoryconverter.RoutePointStamped;

/**
 * Interface for a ConflictManager
 * A ConflictManager extends the behavior of a ConflictDetector 
 * by providing an interface for adding or removing current vehicle paths to the conflict detection system
 */
public interface IConflictManager extends IConflictDetector{
  /**
   * Adds a path to the ConflictManager generated from a MobilityPath message
   * The provided path will be associated with the provided vehicleStaticId from a MobilityPath message
   * 
   * @param path The path to add for future conflict detection
   * @param vehicleStaticId The static id of the vehicle who will be following this path
   * 
   * @return True if the path could be added. False if not.
   */
  boolean addMobilityPath(List<RoutePointStamped> path, String vehicleStaticId);

  /**
   * Adds a path to the ConflictManager generated from a MobilityRequest message
   * The provided path will be associated with the provided planId from a MobilityRequest message
   * 
   * @param path The path to add for future conflict detection
   * @param planId The plan id of MobilityRequest message used to generate this path
   * @param vehicleStaticId The static vehicle mobility id
   * 
   * @return True if the path could be added. False if not.
   */
  boolean addRequestedPath(List<RoutePointStamped> path, String planId, String vehicleStaticId);

  /**
   * Removes from the ConflictManager the MobilityPath belonging to the associated VehicleStaticId
   * 
   * @param vehicleStaticId The static vehicle mobility id which identifies the path to remove
   * 
   * @return True if the path could be removed. False if not.
   */
  boolean removeMobilityPath(String vehicleStaticId); 

  /**
   * Removes from the ConflictManager the MobilityRequest path with the corresponding plan id
   * 
   * @param planIdThe plan id of MobilityRequest message which identifies the path to remove
   * 
   * @return True if the path could be removed. False if not.
   */
  boolean removeRequestedPath(String planId); 
}