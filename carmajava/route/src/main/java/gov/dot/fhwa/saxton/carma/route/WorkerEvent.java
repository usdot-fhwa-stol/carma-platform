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
package gov.dot.fhwa.saxton.carma.route;

/**
 * A description of the events which can be trigger a state transitions in a RouteWorker
 */
public enum WorkerEvent {
  /**
   * New route data has been loaded
   */
  FILES_LOADED,
  /**
   * A route has been selected
   */
  ROUTE_SELECTED,
  /**
   * A route has been completed
   */
  ROUTE_COMPLETED,
  /**
   * The host vehicle is wandered too far from a route
   */
  LEFT_ROUTE,
  /**
   * A system ready message has been received
   */
  SYSTEM_READY,
  /**
   * A system failure message has been received
   */
  SYSTEM_FAILURE
}
