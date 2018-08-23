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
  public List<RoutePointStamped> interpolateMotion(List<Node> trajectory, double distanceStep) {
    Node prevNode = null;
    List<RoutePointStamped> points = new LinkedList<>();

    // Iterate over list of nodes
    for (Node n: trajectory) {
      if (prevNode == null) {

        points.add(new RoutePointStamped(n.getDistanceAsDouble(), 0, n.getTimeAsDouble())); // Add the first node to the list
        prevNode = n;
        continue;
      }

      final double t_0 = prevNode.getTimeAsDouble();
      final double x_0 = prevNode.getDistanceAsDouble();
      final double v_0 = prevNode.getSpeedAsDouble();

      final double t_f = n.getTimeAsDouble();
      final double v_f = n.getSpeedAsDouble();
      final double x_f = n.getDistanceAsDouble();

      final double dt =  t_f - t_0;
      final double dv = v_f - v_0;
      final double dx = x_f - x_0;
      
      if (dx < distanceStep) {
        throw new IllegalArgumentException("interpolateMotion distance step size of "
         + distanceStep + " is less than gab between nodes " + prevNode + " " + n);
      }

      final double a = dv / dt; // Assume constant acceleration
      final double two_a_x = 2 * a * distanceStep;
      
      // double v = v_0;
      // double x = x_0;
      // double t = t_0;

      double v_old = v_0;
      double v = Math.sqrt(v_old * v_old + two_a_x);
      double t = t_0 + (v_old - v) / a;
      double x = x_0 + distanceStep;

      // Interpolate between current node and previous node
      while (x < x_f) {

        points.add(new RoutePointStamped(x, 0, t)); // Add previous node and the new interpolated nodes to list
        v_old = v;
        v = Math.sqrt(v_old * v_old + two_a_x);
        t += (v_old - v) / a;
        x += distanceStep;
      }

      // Add the current node to the list
      points.add(new RoutePointStamped(x_f, 0, t_f));

      prevNode = n;
    }

    return points;
  }

}
