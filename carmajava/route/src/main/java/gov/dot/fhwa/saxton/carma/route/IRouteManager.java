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

package gov.dot.fhwa.saxton.carma.route;

import cav_msgs.*;
import org.ros.message.Time;
import org.ros.rosjava_geometry.Transform;

/**
 * Interface defines the needed functions of a route worker
 */
public interface IRouteManager {

  /**
   * Publishes the active route in the form of a ros message
   *
   * @param route The active route message
   */
  void publishActiveRoute(cav_msgs.Route route);

  /**
   * Publishes the current route following state in the form of a ros message
   *
   * @param routeState  The current route state
   */
  void publishRouteState(RouteState routeState);

  /**
   * Publishes the latest route event in the form of a ros message
   * 
   * @param routeEvent The latest route event
   */
  void publishRouteEvent(RouteEvent routeEvent);

  /**
   * Gets the transform of between the requested frames
   */
  Transform getTransform(String parentFrame, String childFrame, Time stamp);
  
  /**
   * Gets the current time
   *
   * @return The time
   */
  Time getTime();

  /**
   * Safely shutdown the node
   */
  void shutdown();
}
