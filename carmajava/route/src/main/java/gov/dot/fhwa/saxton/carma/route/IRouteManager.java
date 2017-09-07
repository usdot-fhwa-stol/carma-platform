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

import cav_msgs.*;
import cav_msgs.Route;
import cav_srvs.GetAvailableRoutesResponse;
import cav_srvs.SetActiveRouteRequest;
import cav_srvs.SetActiveRouteResponse;
import org.ros.message.Time;

import java.util.PriorityQueue;

/**
 * Interface defines the needed functions of a route worker
 */
public interface IRouteManager {

  /**
   * Publishes a system alert message
   *
   * @param systemAlert  system alert messages
   */
  void publishSystemAlert(SystemAlert systemAlert);

  /**
   * Publishes a ros message corresponding to the current route segment
   *
   * @param routeSegment The current route segment message
   */
  void publishCurrentRouteSegment(cav_msgs.RouteSegment routeSegment);

  /**
   * Publishes the active route in the form of a ros message
   *
   * @param route The active route message
   */
  void publishActiveRoute(Route route);

  /**
   * Publishes the current route following state in the form of a ros message
   *
   * @param routeState  The current route state
   */
  void publishRouteState(RouteState routeState);

  /**
   * Gets the current time
   *
   * @return The time
   */
  Time getTime();
}
