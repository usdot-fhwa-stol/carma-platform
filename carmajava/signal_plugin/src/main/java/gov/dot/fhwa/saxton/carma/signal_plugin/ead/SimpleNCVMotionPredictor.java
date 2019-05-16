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

package gov.dot.fhwa.saxton.carma.signal_plugin.ead;

import java.util.LinkedList;
import java.util.List;

import cav_msgs.RoadwayObstacle;
import gov.dot.fhwa.saxton.carma.guidance.util.trajectoryconverter.RoutePointStamped;

/**
 * The SimpleNCVMotionPredictor provides an implementation of IMotionPredictor using average vehicle speed to predict future detected vehicle motions.
 * The returned list of RoutePointStamped objects can be used to check conflicts with the CARMA conflict detection system
 */
public class SimpleNCVMotionPredictor implements IMotionPredictor {
	
  protected static double FLOATING_POINT_EPSILON = 0.000001; 
  protected static double RADAR_DETECTION_NOISE  = 1.0;

   @Override
  public List<RoutePointStamped> predictMotion(String objId, List<RoadwayObstacle> objTrajectory, double distanceStep, double timeDuration) {

    List<RoutePointStamped> projection = new LinkedList<>();

    // Return an empty list if provided with an empty list
    if (objTrajectory.isEmpty()) {
      return projection; 
    }

    // Calculate average speed from historical data
    double averageSpeed = objTrajectory.stream()
    		                       .mapToDouble(a -> a.getObject().getVelocity().getTwist().getLinear().getX())
    		                       .average()
    		                       .getAsDouble();

    // Generate projection using average speed
    // Set endtime as the last timestamp in the provided history plus the time duration 
    double startDist = objTrajectory.get(objTrajectory.size() - 1).getDownTrack();
    double d = startDist;
    double startTime = objTrajectory.get(objTrajectory.size() - 1).getObject().getHeader().getStamp().toSeconds();
    double t = startTime;
    double endTime = timeDuration + startTime;
    double endDist = startDist + timeDuration * averageSpeed;

    // Add the initial point
    projection.add(new RoutePointStamped(d, 0, t));
    if (averageSpeed < RADAR_DETECTION_NOISE) { // If the m/s is smaller than 1 m/s (2.23694 mph) then assume the vehicle is stopped
      // Use small time increments instead of distance steps
      while (t + FLOATING_POINT_EPSILON < endTime) {
    	t += 0.1;
        projection.add(new RoutePointStamped(d, 0, t)); // Assume that the vehicle is stationary
      }
    } else { // Vehicle is in motion so use distance steps
      double timeStep = distanceStep / averageSpeed;
      while (d + FLOATING_POINT_EPSILON < endDist) {
        d += distanceStep;        
        t += timeStep;
        projection.add(new RoutePointStamped(d, 0, t));
      }
    }

    return projection;
  }
}
