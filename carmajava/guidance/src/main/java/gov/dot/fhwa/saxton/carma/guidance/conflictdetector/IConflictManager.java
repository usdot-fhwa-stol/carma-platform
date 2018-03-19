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
   * Adds a path to the ConflictManager
   * The provided path will be associated with the provided vehicleStaticId from a MobilityPath message
   * 
   * @param path The path to add for future conflict detection
   * @param vehicleStaticId The static id of the vehicle who will be following this path
   * 
   * @return True if the path could be added. False if not.
   */
  boolean addPath(List<RoutePointStamped> path, String vehicleStaticId);

  /**
   * Removes from the ConflictManager the path belonging to the associated VehicleStaticId
   * 
   * @param vehicleStaticId The static vehicle mobility id which identifies the path to remove
   * 
   * @return True if the path could be removed. False if not.
   */
  boolean removePath(String vehicleStaticId); 
}