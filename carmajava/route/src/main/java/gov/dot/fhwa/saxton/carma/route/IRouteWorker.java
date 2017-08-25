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

import cav_msgs.SystemAlert;
import cav_srvs.GetAvailableRoutesResponse;
import cav_srvs.SetActiveRouteRequest;
import cav_srvs.SetActiveRouteResponse;
import org.ros.message.Time;
import java.util.PriorityQueue;

/**
 * Interface defines the needed functions of a route worker
 */
public interface IRouteWorker {
  /**
   * Provides a response for a the GetAvailableRoutes service
   *
   * @return Response message to be sent back
   */
  GetAvailableRoutesResponse getAvailableRoutes();

  /**
   * Selects a active route by id as the route to be tracked
   *
   * @param request The service request message containing the route id
   * @return Response message to be sent back
   */
  SetActiveRouteResponse setActiveRoute(SetActiveRouteRequest request);

  /**
   * Function for used in NavSatFix topic callback. Used to update vehicle location with the provided data
   *
   * @param msg The NavSatFix message containing the host vehicle location
   */
  void handleNavSatFixMsg(sensor_msgs.NavSatFix msg);

  /**
   * Function for used in SystemAlert topic callback.
   *
   * @param msg The system alert message
   */
  void handleSystemAlertMsg(cav_msgs.SystemAlert msg);

  //  /**  TODO: Add once we have tim messages
  //   * Function for used in Tim topic callback. Used to update waypoints on a route
  //   * @param msg The tim message
  //   */
  //   void handleTimMsg(cav_msgs.Tim msg);

  /**
   * Provides a queue of system alert messages which should be published by a ros node.
   * Alerts are organized in order of severity with FATAL alerts having highest priority
   *
   * @return Queue of system alert messages
   */
  PriorityQueue<SystemAlert> getSystemAlertTopicMsgs();

  /**
   * Gets a ros message corresponding to the current route segment
   *
   * @return The current route segment message
   */
  cav_msgs.RouteSegment getCurrentRouteSegmentTopicMsg();

  /**
   * Gets the active route in the form of a ros message
   *
   * @return The active route message
   */
  cav_msgs.Route getActiveRouteTopicMsg();

  /**
   * Gets the current route following state in the form of a ros message
   *
   * @param seq  The current sequence count which will be used in the header of this message
   * @param time The current time which will be used in the header of this message.
   * @return the route state message
   */
  cav_msgs.RouteState getRouteStateTopicMsg(int seq, Time time);

  /**
   * Gets the current state of a route worker
   *
   * @return The worker state
   */
  WorkerState getState();
}
