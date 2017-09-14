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
 * A description of states which a route worker can take
 */
public enum WorkerState {
  /**
   * Starting state. No routes have been loaded yet
   */
  LOADING_ROUTES,
  /**
   * Routes have been loaded and waiting for selection
   */
  ROUTE_SELECTION,
  /**
   * A route has been selected and is waiting to be started
   */
  WAITING_TO_START,
  /**
   * The vehicle is actively following a route
   */
  FOLLOWING_ROUTE
}
