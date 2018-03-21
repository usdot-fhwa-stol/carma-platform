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

package gov.dot.fhwa.saxton.carma.guidance.util.trajectoryconverter;

import gov.dot.fhwa.saxton.carma.guidance.maneuvers.FutureLateralManeuver;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.FutureLongitudinalManeuver;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.IComplexManeuver;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.LateralManeuver;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.LongitudinalManeuver;
import gov.dot.fhwa.saxton.carma.guidance.trajectory.Trajectory;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;

import org.ros.message.MessageFactory;
import org.ros.rosjava_geometry.Transform;
import org.ros.rosjava_geometry.Vector3;

import cav_msgs.LocationECEF;
import cav_msgs.LocationOffsetECEF;
import cav_msgs.Route;
import cav_msgs.RouteSegment;
import cav_msgs.RouteState;
import gov.dot.fhwa.saxton.carma.geometry.cartesian.Point2D;
import gov.dot.fhwa.saxton.carma.geometry.cartesian.Point3D;

/**
 * Interface for objects responsible for converting Trajectories into paths described as a sequence for ECEF points for use in Mobility Messages
 * Users of this interface should call the convertToPath function
 * then call the pathToMessage function to convert the path into a MobilityPath message as needed.
 * 
 * Paths are described internally as a list of ECEFPointStamped objects
 * Simple Longitudinal Maneuver motion is estimated using the basic kinematic equations of motion
 * Lateral motion is approximated using a cubic function calculated based on the start and end distance of a maneuver
 * Complex maneuvers are treated as stead speed maneuvers operating at the average of the min and max speeds of that maneuver
 * 
 * Crosstrack distance is preserved across all traversed route segments
 * Note: This class will become less reliable when dealing with lane changes along tight curves and changing lane widths
 */
public interface ITrajectoryConverter {

  /**
   * Converts the provided trajectory and starting configuration into a list of (downtrack, crosstrack) points with associated time stamps
   * 
   * This function determines all the point downtrack distances using simple longitudinal maneuvers and kinematic equations.
   * Then the longitudinal maneuvers are used to shift the crosstrack values of each point
   * Then any complex maneuvers are added to the path
   * Finally all points are converted into the ECEF frame
   * 
   * @param traj The trajectory to convert
   * @param currentTimeMs The starting time for this path in ms 
   * @param route A route message containing all possible route segments
   * @param routeState A route state message containing the current segment and progress along the route
   * 
   * @return A list of downtrack, crosstrack points associated with time stamps and segments
   */
  List<RoutePointStamped> convertToPath(Trajectory traj, long startTimeMS, Route route, RouteState routeState);
  /**
   * Helper function for converting a List of RoutePoint2DStamped into List of ECEFPointStamped
   * 
   * @param path The list of RoutePoint2DStamped to be converted
   * 
   * @return The path described as ECEF points
   */
  List<ECEFPointStamped> toECEFPoints(List<RoutePointStamped> path);

   /**
   * Helper function for converting a List of ECEFPointStamped into List of RoutePointStamped
   * 
   * @param path The list of ECEFPointStamped to be converted
   * @param route The current route
   * @param routeState The current route state
   * 
   * @return The path described as points along a route
   */
  List<RoutePointStamped> toRoutePointStamped(List<ECEFPointStamped> path, cav_msgs.Route route, cav_msgs.RouteState routeState);

  /**
   * Function converts a path to a cav_msgs.Trajectory message using the provided message factory
   * 
   * @param path The list of ecef points and times which defines the path
   * @param messageFactory The message factory which will be used to build this message
   * 
   * @return A cav_msgs.Trajectory message. This message will be empty if the path was empty
   */
   cav_msgs.Trajectory pathToMessage(List<RoutePointStamped> path, MessageFactory messageFactory);

  /**
   * Function which converts and individual Simple Longitudinal Maneuver to a path based on starting configuration
   * This function is used internally in the convertToPath function
   * 
   * @param maneuver The maneuver to convert
   * @param path The list which will store the generated points
   * @param startingData The starting configuration of the vehicle
   * @param route The route the vehicle is on
   */
  LongitudinalSimulationData addLongitudinalManeuverToPath(
    final LongitudinalManeuver maneuver, List<RoutePointStamped> path,
    final LongitudinalSimulationData startingData, final Route route);
}