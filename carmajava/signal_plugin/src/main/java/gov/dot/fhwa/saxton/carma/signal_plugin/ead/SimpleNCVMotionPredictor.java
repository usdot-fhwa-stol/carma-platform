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

package gov.dot.fhwa.saxton.carma.signal_plugin.ead;

import java.util.LinkedList;
import java.util.List;

import cav_msgs.RoadwayObstacle;
import gov.dot.fhwa.saxton.carma.guidance.util.trajectoryconverter.RoutePointStamped;

/**
 * The SimpleNCVMotionPredictor provides an implementation of IMotionPredictor using simple linear regression to predict future detected vehicle motions.
 * The returned list of RoutePointStamped objects can be used to check conflicts with the CARMA conflict detection system
 * 
 * Simple Linear Regression
 * 
 * Equation of a line -> y = a*x + b
 * Where 
 * x is the independent variable
 * y is the dependent variable
 * a is the slope of the line
 * b is the intercept of the line
 * 
 * The simple linear regression equation is as follows
 * 
 * b = (Ey * E(x^2) - Ex * E(xy)) / (n * E(x^2) - (Ex)^2);
 * a = (n * E(xy) - Ex * Ey) / (n * E(x^2) - (Ex)^2);
 * 
 * Where
 * n = number of samples in data set
 * E represents the summation symbol Sigma
 */
public class SimpleNCVMotionPredictor implements IMotionPredictor {

   @Override
  public List<RoutePointStamped> predictMotion(String objId, List<RoadwayObstacle> objTrajectory, double distanceStep, double timeDuration) {

    List<RoutePointStamped> projection = new LinkedList<>();

    // Return an empty list if provided with less than 2 points
    if (objTrajectory.size() < 2) {
      return projection; 
    }

    double sumDist = 0; // Since we are using distance steps the independent variable will be distance
    double sumTime = 0; // Since we are using distance steps the dependent variable will be time
    double sumSqrDist = 0;
    double sumDistxTime = 0;

    // Calculate sums for regression
    for (RoadwayObstacle n: objTrajectory) {
      
      double d = n.getDownTrack();
      double t = n.getObject().getHeader().getStamp().toSeconds();
      double d_sqr = d * d;
      double td = t * d; 

      sumDist += d;
      sumTime += t;
      sumSqrDist += d_sqr;
      sumDistxTime += td; 
    }

    // Compute intercept and slope
    double count = objTrajectory.size();
    double denominator = (count * sumSqrDist - sumDist * sumDist);

    double intercept = (sumTime * sumSqrDist - sumDist * sumDistxTime) / denominator;
    double slope = (count * sumDistxTime - sumDist * sumTime) / denominator;

    // Generate projection using regression model
    // Set endtime as the last timestamp in the provided history plus the time duration 
    double startDist = objTrajectory.get(objTrajectory.size() - 1).getDownTrack();
    double d = startDist;
    double t = objTrajectory.get(objTrajectory.size() - 1).getObject().getHeader().getStamp().toSeconds();
    double endTime = timeDuration + t;
    double endDistance = (endTime - intercept) / slope;

    if (slope > 1.0) { // If the s/m is greater than 1 m/s (2.23694 mph) then assume the vehicle is stopped
      // Use small time increments instead of distance steps
      while (t < endTime) {

        t += 0.1;
        projection.add(new RoutePointStamped(d, 0, t)); // Assume that the vehicle is stationary
      }
    } else { // Vehicle is in motion so use distance steps
      while (d < endDistance) {

        d += distanceStep;
        projection.add(new RoutePointStamped(d, 0, d * slope + intercept));
      }
    }

    return projection;
  }
}
