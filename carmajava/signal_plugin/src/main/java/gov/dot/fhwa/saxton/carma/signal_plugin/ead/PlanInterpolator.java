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
 * 
 * NOTE: Returned route points do not have valid lane, crosstrack, or route segment index
 */
public class PlanInterpolator implements IMotionInterpolator {

  @Override
  public List<RoutePointStamped> interpolateMotion(List<Node> trajectory, double timeStep) {
    Node prevNode = null;
    List<RoutePointStamped> points = new LinkedList<>();
    final double timeStep_sqr = timeStep * timeStep;

    // Iterate over list of nodes
    for (Node n: trajectory) {
      if (prevNode == null) {
        // Add the first node to the list and store it as the prev node
        points.add(new RoutePointStamped(n.getDistanceAsDouble(), 0, n.getTimeAsDouble()));
        prevNode = n;
        continue;
      }

      final double t_0 = prevNode.getTimeAsDouble();
      final double x_0 = prevNode.getDistanceAsDouble();
      final double v_0 = prevNode.getSpeedAsDouble();

      final double t_f = n.getTimeAsDouble();

      final double dt =  t_f - t_0;
      
      if (dt < timeStep) {
        throw new IllegalArgumentException("interpolateMotion timestep size of "
         + timeStep + " is less than gab between nodes " + prevNode + " " + n);
      }

      final double dv = n.getSpeedAsDouble() - v_0;
      final double a = dv / dt; // Assume constant acceleration
      final double half_a = a * 0.5;
      final double dvPerTimestep = a*timeStep;
      
      double v = v_0;
      double x = x_0;
      double t = t_0;

      // Interpolate between current node and previous node
      while (t < t_f) {

        x += half_a * timeStep_sqr + v * timeStep;
        v += dvPerTimestep;
        t += timeStep;
        points.add(new RoutePointStamped(x, 0, t)); // Add new interpolated node to list
      }

      prevNode = n;
    }

    return points;
  }

}
