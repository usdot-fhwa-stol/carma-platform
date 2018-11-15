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

import gov.dot.fhwa.saxton.carma.guidance.util.ITimeProvider;
import gov.dot.fhwa.saxton.carma.guidance.util.trajectoryconverter.RoutePointStamped;
import gov.dot.fhwa.saxton.carma.signal_plugin.ead.trajectorytree.Node;

/**
 * A MotionInterpolator is responsible for interpolating the position of the host vehicle between two or more nodes in a plan
 * The returned list of RoutePointStamped objects can be used to check conflicts with the CARMA conflict detection system
 * 
 * NOTE: Returned route points do not have valid lane, crosstrack, or route segment index
 */
public class PlanInterpolator implements IMotionInterpolator {

  double MIN_ACCELERATION = 0.000001;
  @Override
  public List<RoutePointStamped> interpolateMotion(List<Node> trajectory, double distanceStep, double timeOffset, double distanceOffset) {
    Node prevNode = null;
    List<RoutePointStamped> points = new LinkedList<>();

    // Iterate over list of nodes
    for (Node n: trajectory) {
      if (prevNode == null) {

        points.add(new RoutePointStamped(n.getDistanceAsDouble() + distanceOffset, 0, n.getTimeAsDouble() + timeOffset)); // Add the first node to the list
        prevNode = n;
        continue;
      }

      final double t_0 = prevNode.getTimeAsDouble() + timeOffset;
      final double x_0 = prevNode.getDistanceAsDouble() + distanceOffset;
      final double v_0 = prevNode.getSpeedAsDouble();

      final double t_f = n.getTimeAsDouble() + timeOffset;
      final double v_f = n.getSpeedAsDouble();
      final double x_f = n.getDistanceAsDouble() + distanceOffset;

      final double dt = t_f - t_0;
      final double dv = v_f - v_0;
      final double dx = x_f - x_0;
      
      // If the distance is less than our distance step we will directly add the current point to our path with no interpolation
      if (dx < distanceStep) {
        points.add(new RoutePointStamped(x_f, 0, t_f));
        continue;
      }

      final double a = dv / dt; // Assume constant acceleration
      final double two_a_x = 2 * a * distanceStep;

      double v_old = v_0;
      double v = v_old; 
      double t;
      double x = x_0 + distanceStep;
      
      if (Math.abs(a) < MIN_ACCELERATION) {
        t = t_0 + distanceStep / v; // With constant speed 

        // Interpolate between current node and previous node
        while (x < x_f) {

          points.add(new RoutePointStamped(x, 0, t)); // Add previous node and the new interpolated nodes to list
          t += distanceStep / v;
          x += distanceStep;
        }

      } else {
        double v_f_sqr = v_old * v_old + two_a_x;
        if (v_f_sqr <= 0.0) {
          v = 0.0;
        } else {
          v = Math.sqrt(v_f_sqr);
        }
        t = t_0 + (v - v_old) / a; // With constant acceleration

        // Interpolate between current node and previous node
        while (x < x_f) {

          points.add(new RoutePointStamped(x, 0, t)); // Add previous node and the new interpolated nodes to list

          v_old = v;
          v_f_sqr = v_old * v_old + two_a_x;
          if (v_f_sqr <= 0.0) {
            v = 0.0;
          } else {
            v = Math.sqrt(v_f_sqr);
          }
          t += (v - v_old) / a;
          x += distanceStep;
        }
      }

      // Add the current node to the list
      points.add(new RoutePointStamped(x_f, 0, t_f));

      prevNode = n;
    }

    return points;
  }

}
