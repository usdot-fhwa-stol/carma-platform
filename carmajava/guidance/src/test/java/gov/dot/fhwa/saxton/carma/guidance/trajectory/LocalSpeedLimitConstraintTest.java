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

package gov.dot.fhwa.saxton.carma.guidance.trajectory;

import org.junit.Before;
import org.junit.Test;
import org.junit.Ignore;
import org.ros.message.MessageFactory;
import org.ros.message.Time;
import org.ros.node.NodeConfiguration;
import std_msgs.Header;

import static org.junit.Assert.*;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.Mockito.*;

import cav_msgs.Route;
import cav_msgs.RouteSegment;
import cav_msgs.RouteWaypoint;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.ISimpleManeuver;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.LongitudinalManeuver;
import gov.dot.fhwa.saxton.carma.guidance.util.ILogger;
import gov.dot.fhwa.saxton.carma.guidance.util.ILoggerFactory;
import gov.dot.fhwa.saxton.carma.guidance.util.LoggerManager;

import java.util.ArrayList;
import java.util.List;

public class LocalSpeedLimitConstraintTest {
  private NodeConfiguration nodeConfig = NodeConfiguration.newPrivate();
  private MessageFactory messageFactory = nodeConfig.getTopicMessageFactory();

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

  @Before
  public void setup() {
	
	ILoggerFactory mockFact = mock(ILoggerFactory.class);
    ILogger mockLogger = mock(ILogger.class);
    when(mockFact.createLoggerForClass(any())).thenReturn(mockLogger);
    LoggerManager.setLoggerFactory(mockFact);
      
    List<Double> speeds = new ArrayList<>();
    speeds.add(10.0);
    speeds.add(20.0);
    speeds.add(30.0);
    route = generateRouteWithSpeedLimits(speeds, 10.0);
    lslc = new LocalSpeedLimitConstraint(route);
  }

  @Test
  public void testSuccess() {
    LongitudinalManeuver m1 = mock(LongitudinalManeuver.class);
    when(m1.getStartDistance()).thenReturn(0.0);
    when(m1.getEndDistance()).thenReturn(10.0);
    when(m1.getStartSpeed()).thenReturn(0.0);
    when(m1.getTargetSpeed()).thenReturn(9.0);

    LongitudinalManeuver m2 = mock(LongitudinalManeuver.class);
    when(m2.getStartDistance()).thenReturn(10.0);
    when(m2.getEndDistance()).thenReturn(20.0);
    when(m2.getStartDistance()).thenReturn(10.0);
    when(m2.getEndDistance()).thenReturn(10.0);

    LongitudinalManeuver m3 = mock(LongitudinalManeuver.class);
    when(m3.getStartDistance()).thenReturn(20.0);
    when(m3.getEndDistance()).thenReturn(30.0);
    when(m3.getStartDistance()).thenReturn(10.0);
    when(m3.getEndDistance()).thenReturn(21.0);

    lslc.visit(m1);
    lslc.visit(m2);
    lslc.visit(m3);

    assertTrue(lslc.getResult().getSuccess());
  }

  @Test
  public void testSuccessMixedTypes() {
    LongitudinalManeuver m1 = mock(LongitudinalManeuver.class);
    when(m1.getStartDistance()).thenReturn(0.0);
    when(m1.getEndDistance()).thenReturn(9.0);
    when(m1.getStartSpeed()).thenReturn(0.0);
    when(m1.getTargetSpeed()).thenReturn(1.0);

    ISimpleManeuver m2 = mock(ISimpleManeuver.class);
    when(m2.getStartDistance()).thenReturn(0.0);
    when(m2.getEndDistance()).thenReturn(10.0);

    LongitudinalManeuver m3 = mock(LongitudinalManeuver.class);
    when(m3.getStartSpeed()).thenReturn(9.0);
    when(m3.getTargetSpeed()).thenReturn(19.0);
    when(m3.getStartDistance()).thenReturn(10.0);
    when(m3.getEndDistance()).thenReturn(20.0);

    lslc.visit(m1);
    lslc.visit(m2);
    lslc.visit(m3);

    assertTrue(lslc.getResult().getSuccess());
  }

  @Test
  public void testRejection() {
    LongitudinalManeuver m1 = mock(LongitudinalManeuver.class);
    when(m1.getStartDistance()).thenReturn(0.0);
    when(m1.getEndDistance()).thenReturn(10.0);
    when(m1.getStartSpeed()).thenReturn(0.0);
    when(m1.getTargetSpeed()).thenReturn(10.0);

    LongitudinalManeuver m2 = mock(LongitudinalManeuver.class);
    when(m2.getStartDistance()).thenReturn(10.0);
    when(m2.getEndDistance()).thenReturn(20.0);
    when(m2.getStartSpeed()).thenReturn(10.0);
    when(m2.getTargetSpeed()).thenReturn(35.0);

    lslc.visit(m1);
    lslc.visit(m2);

    assertFalse(lslc.getResult().getSuccess());
  }

  @Test
  public void testRejection2() {
    LongitudinalManeuver m1 = mock(LongitudinalManeuver.class);
    when(m1.getStartDistance()).thenReturn(0.0);
    when(m1.getEndDistance()).thenReturn(10.0);
    when(m1.getStartSpeed()).thenReturn(21.0);
    when(m1.getTargetSpeed()).thenReturn(25.0);

    LongitudinalManeuver m2 = mock(LongitudinalManeuver.class);
    when(m2.getStartDistance()).thenReturn(10.0);
    when(m2.getEndDistance()).thenReturn(20.0);
    when(m2.getStartSpeed()).thenReturn(35.0);
    when(m2.getTargetSpeed()).thenReturn(10.0);

    LongitudinalManeuver m3 = mock(LongitudinalManeuver.class);
    when(m3.getStartDistance()).thenReturn(20.0);
    when(m3.getEndDistance()).thenReturn(30.0);
    when(m3.getStartSpeed()).thenReturn(10.0);
    when(m3.getTargetSpeed()).thenReturn(40.0);

    lslc.visit(m1);
    lslc.visit(m2);
    lslc.visit(m3);

    assertFalse(lslc.getResult().getSuccess());
  }

  protected LocalSpeedLimitConstraint lslc;
  protected Route route;
}
