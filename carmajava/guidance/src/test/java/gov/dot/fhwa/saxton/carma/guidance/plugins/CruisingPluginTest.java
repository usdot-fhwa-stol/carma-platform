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

package gov.dot.fhwa.saxton.carma.guidance.plugins;

import org.apache.commons.logging.Log;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import org.ros.message.MessageFactory;
import org.ros.message.Time;
import org.ros.node.NodeConfiguration;
import std_msgs.Header;
import java.util.ArrayList;
import java.util.List;

import static org.junit.Assert.*;
import static org.mockito.Mockito.*;

import cav_msgs.Route;
import cav_msgs.RouteSegment;
import cav_msgs.RouteWaypoint;
import gov.dot.fhwa.saxton.carma.guidance.ArbitratorService;
import gov.dot.fhwa.saxton.carma.guidance.ManeuverPlanner;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.IManeuver;
import gov.dot.fhwa.saxton.carma.guidance.params.ParameterSource;
import gov.dot.fhwa.saxton.carma.guidance.plugins.CruisingPlugin.SpeedLimit;
import gov.dot.fhwa.saxton.carma.guidance.plugins.CruisingPlugin.TrajectorySegment;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.IPubSubService;
import gov.dot.fhwa.saxton.carma.guidance.trajectory.Trajectory;

public class CruisingPluginTest {

  private NodeConfiguration nodeConfig = NodeConfiguration.newPrivate();
  private MessageFactory messageFactory = nodeConfig.getTopicMessageFactory();

  @Before
  public void setup() {
    PluginServiceLocator psl = new PluginServiceLocator(mock(ArbitratorService.class),
        mock(PluginManagementService.class), mock(IPubSubService.class), mock(ParameterSource.class),
        mock(ManeuverPlanner.class), mock(Log.class));
    cruise = new CruisingPlugin(psl);
  }

  private byte mpsToMph(double mps) {
    return (byte) (mps * 2.23694);
  }

  private Route generateRouteWithSpeedLimits(List<Double> speeds, double segLength) {
    Route route = messageFactory.newFromType(Route._TYPE);
    Header header = messageFactory.newFromType(Header._TYPE);
    header.setFrameId("0");
    header.setSeq(0);
    header.setStamp(Time.fromMillis(System.currentTimeMillis()));

    route.setHeader(header);
    route.setRouteID("test-route");
    route.setRouteName("test-route");

    List<RouteSegment> segments = new ArrayList<>();
    List<RouteWaypoint> waypoints = new ArrayList<>();

    int curId = 0;
    RouteWaypoint initialWp = messageFactory.newFromType(RouteWaypoint._TYPE);
    initialWp.setWaypointId(curId++);
    initialWp.setLaneCount((byte) 1);
    initialWp.setSpeedLimit(mpsToMph(0.0));
    waypoints.add(initialWp);

    for (Double speed : speeds) {
      RouteWaypoint waypoint = messageFactory.newFromType(RouteWaypoint._TYPE);
      waypoint.setWaypointId(curId++);
      waypoint.setLaneCount((byte) 1);
      waypoint.setSpeedLimit(mpsToMph(speed));
      waypoints.add(waypoint);
    }

    for (int i = 0; i < waypoints.size() - 1; i++) {
      RouteSegment seg = messageFactory.newFromType(RouteSegment._TYPE);
      seg.setLength(segLength);
      seg.setPrevWaypoint(waypoints.get(i));
      seg.setWaypoint(waypoints.get(i + 1));
      segments.add(seg);
    }

    route.setSegments(segments);

    return route;
  }

  @Test
  public void testProcessSpeedLimits() {
    List<Double> speeds = new ArrayList<>();
    speeds.add(1.0);
    speeds.add(2.0);
    speeds.add(3.0);

    Route route = generateRouteWithSpeedLimits(speeds, 1.0);

    List<SpeedLimit> limits = cruise.processSpeedLimits(route);
    for (int i = 0; i < limits.size(); i++) {
      assertEquals(speeds.get(i), limits.get(i).speedLimit, 0.5);
    }
  }

  // We'll only have the CrusingPlugin generating longitudinal maneuvers, so the whole
  // longitudinal trajectory will always be open for now.
  @Test
  public void testFindTrajectoryGaps() {
    Trajectory t = new Trajectory(0.0, 20.0);
    List<TrajectorySegment> gaps = cruise.findTrajectoryGaps(t, 0.0, 5.0);

    assertEquals(1, gaps.size());
    assertEquals(0.0, gaps.get(0).start, 0.01);
    assertEquals(20.0, gaps.get(0).end, 0.01);
    assertEquals(0.0, gaps.get(0).startSpeed, 0.01);
    assertEquals(5.0, gaps.get(0).endSpeed, 0.01);
  }

  // Test to see if a lateral maneuver breaks finding the longitudinal gap
  @Test
  public void testFindTrajectoryGaps2() {
    Trajectory t = new Trajectory(0.0, 20.0);

    IManeuver mockLatManeuver = mock(IManeuver.class);
    when(mockLatManeuver.getStartDistance()).thenReturn(2.0);
    when(mockLatManeuver.getEndDistance()).thenReturn(10.0);

    List<TrajectorySegment> gaps = cruise.findTrajectoryGaps(t, 0.0, 5.0);

    assertEquals(1, gaps.size());
    assertEquals(0.0, gaps.get(0).start, 0.01);
    assertEquals(20.0, gaps.get(0).end, 0.01);
    assertEquals(0.0, gaps.get(0).startSpeed, 0.01);
    assertEquals(5.0, gaps.get(0).endSpeed, 0.01);
  }

  @Test
  public void testGetSpeedLimits() {
    List<Double> speeds = new ArrayList<>();
    speeds.add(1.0);
    speeds.add(2.0);
    speeds.add(3.0);
    speeds.add(4.0);
    speeds.add(5.0);

    Route route = generateRouteWithSpeedLimits(speeds, 5.0);

    List<SpeedLimit> limits = cruise.processSpeedLimits(route);
    List<SpeedLimit> filteredLimits = cruise.getSpeedLimits(limits, 5.5, 15.5);

    assertEquals(2, filteredLimits.size());
    assertEquals(3.0, filteredLimits.get(0).speedLimit, 0.5);
    assertEquals(4.0, filteredLimits.get(1).speedLimit, 0.5);
    assertEquals(10.0, filteredLimits.get(0).location, 0.01);
    assertEquals(15.0, filteredLimits.get(1).location, 0.01);
  }

  private CruisingPlugin cruise;
}