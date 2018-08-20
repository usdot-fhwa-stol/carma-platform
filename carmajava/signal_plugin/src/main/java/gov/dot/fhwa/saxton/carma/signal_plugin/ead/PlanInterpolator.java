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
public class PlanInterpolator implements IMotionInterpolator {

  @Override
  public List<RoutePointStamped> interpolateMotion(List<Node> trajectory, double timeStep) {
    Node prevNode = null;

    for (Node n: trajectory) {
      if (prevNode == null) {
        prevNode = n;
        continue;
      }

      double t_0 = prevNode.getTimeAsDouble();
      double dt = n.getTimeAsDouble() - prevNode.getTimeAsDouble();
      double dv = n.getSpeedAsDouble() - prevNode.getSpeedAsDouble();
      double dx = n.getDistanceAsDouble() - prevNode.getDistanceAsDouble();

      double a = dv / dt;

      double t = t_0;
      List<RoutePointStamped> points = new LinkedList<>();
      while (t < t_0) {

        RoutePointStamped rp = new RoutePointStamped(downtrack, crosstrack, time)
        t += timeStep;
      }
    }

    return null;
  }

}