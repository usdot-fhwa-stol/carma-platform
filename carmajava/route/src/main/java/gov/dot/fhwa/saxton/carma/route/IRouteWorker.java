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

import java.util.List;
import java.util.PriorityQueue;

/**
 * Interface defines the needed functions of a route manager
 */
public interface IRouteWorker {
  /**
   * Provides a list of all the routes which are currently available for use.
   * @return A list of route objects //TODO change comment
   */
  GetAvailableRoutesResponse getAvailableRoutes();

  /**
   * Selects a active route by id as the route to be tracked
   * @param request route id //TODO change comment
   */
   SetActiveRouteResponse setActiveRoute(SetActiveRouteRequest request);

  /**
   *
   * @param msg
   */
   void handleNavSatFixMsg(sensor_msgs.NavSatFix msg);

  /**
   *
   * @param msg
   */
  void handleSystemAlertMsg(cav_msgs.SystemAlert msg);
   //void handleTimMsg(cav_msgs.Tim msg); //TODO: Add once we have tim messages

  /**
   *
   * @return
   */
  PriorityQueue<SystemAlert> getSystemAlertTopicMsgs();

  /**
   *
   * @return
   */
  cav_msgs.RouteSegment getCurrentRouteSegmentTopicMsg();

  /**
   *
   * @return
   */
  cav_msgs.Route getActiveRouteTopicMsg();

  /**
   *
   * @return
   */
  cav_msgs.RouteState getRouteStateTopicMsg(int seq, Time time);

  boolean ();

  boolean isSystemStarted();
}
