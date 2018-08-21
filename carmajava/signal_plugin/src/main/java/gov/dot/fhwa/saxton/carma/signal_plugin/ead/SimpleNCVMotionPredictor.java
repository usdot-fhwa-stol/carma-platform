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

import gov.dot.fhwa.saxton.carma.guidance.util.trajectoryconverter.RoutePointStamped;
import gov.dot.fhwa.saxton.carma.signal_plugin.ead.trajectorytree.Node;

/**
 * A MotionInterpolator is responsible for interpolating the position of the host vehicle between two or more nodes in a plan
 * The returned list of RoutePointStamped objects can be used to check conflicts with the CARMA conflict detection system
 */
public class SimpleNCVMotionPredictor implements IMotionPredictor {


   @Override
  public List<RoutePointStamped> predictMotion(String objId, List<Node> objTrajectory, double timeStep, double timeDuration) {

    List<RoutePointStamped> projection = new LinkedList<>();

    // Return an empty list if provided with less than 2 points
    if (objTrajectory.size() < 2) {
      return projection; 
    }

    double sumTime = 0; // Independent variable is time
    double sumDist = 0; // Dependant variable is distance
    double sumSqrTime = 0;
    double sumTimexDist = 0;

    // Calculate sums for regression
    for (Node n: objTrajectory) {
      
      double d = n.getDistanceAsDouble();
      double t = n.getTimeAsDouble();
      double t_sqr = t * t;
      double td = t * d; 

      sumDist += d;
      sumTime += t;
      sumSqrTime += t_sqr;
      sumTimexDist += td; 
    }

    // Compute intercept and slope
    double count = objTrajectory.size();
    double denominator = (count * sumSqrTime - sumTime * sumTime);

    double intercept = (sumDist * sumSqrTime - sumTime * sumTimexDist) / denominator;
    double slope = (count * sumTimexDist - sumTime * sumDist) / denominator;

    // Generate projection using regression model
    // Set start time as the last timestamp in the provided history plus the timestep 
    double startTime = objTrajectory.get(objTrajectory.size() - 1).getTimeAsDouble();
    double t = timeStep + startTime; 
    double endTime = timeDuration + startTime;
    while (t < endTime) {

      projection.add(new RoutePointStamped(t * slope + intercept, 0, t));
      t += timeStep;
    }

    return projection;
  }
}