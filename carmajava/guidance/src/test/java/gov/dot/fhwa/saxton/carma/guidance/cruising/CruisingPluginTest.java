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

package gov.dot.fhwa.saxton.carma.guidance.cruising;

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
import gov.dot.fhwa.saxton.carma.guidance.IGuidanceCommands;
import gov.dot.fhwa.saxton.carma.guidance.ManeuverPlanner;
import gov.dot.fhwa.saxton.carma.guidance.cruising.CruisingPlugin.TrajectorySegment;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.AccStrategyManager;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.IManeuver;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.IManeuverInputs;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.NoOpAccStrategyFactory;
import gov.dot.fhwa.saxton.carma.guidance.params.ParameterSource;
import gov.dot.fhwa.saxton.carma.guidance.plugins.PluginManagementService;
import gov.dot.fhwa.saxton.carma.guidance.plugins.PluginServiceLocator;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.IPubSubService;
import gov.dot.fhwa.saxton.carma.guidance.trajectory.Trajectory;
import gov.dot.fhwa.saxton.carma.guidance.util.GuidanceRouteService;
import gov.dot.fhwa.saxton.carma.guidance.util.ILogger;
import gov.dot.fhwa.saxton.carma.guidance.util.ILoggerFactory;
import gov.dot.fhwa.saxton.carma.guidance.util.LoggerManager;
import gov.dot.fhwa.saxton.carma.guidance.util.RouteService;
import gov.dot.fhwa.saxton.carma.guidance.util.SpeedLimit;

public class CruisingPluginTest {

    //private NodeConfiguration nodeConfig = NodeConfiguration.newPrivate();
    //private MessageFactory messageFactory = nodeConfig.getTopicMessageFactory();
    private GuidanceRouteService routeService;
    private CruisingPlugin cruise;

    @Before
    public void setup() {
        routeService = mock(GuidanceRouteService.class);
        PluginServiceLocator psl = new PluginServiceLocator(mock(ArbitratorService.class),
                mock(PluginManagementService.class), mock(IPubSubService.class), mock(ParameterSource.class),
                new ManeuverPlanner(mock(IGuidanceCommands.class), mock(IManeuverInputs.class)), routeService);
        cruise = new CruisingPlugin(psl);
    }

    //Test if CP can find the right longitudinal gap in empty trajectory
    @Test
    public void testFindTrajectoryGapsWithLongManeuverAtTheEnd() {
        Trajectory t = new Trajectory(0.0, 50.0);
        List<TrajectorySegment> gaps = cruise.findTrajectoryGaps(t, 3.0);
        assertEquals(1, gaps.size());
        assertEquals(0.0, gaps.get(0).startLocation, 0.01);
        assertEquals(50.0, gaps.get(0).endLocation, 0.01);
        assertEquals(3.0, gaps.get(0).startSpeed, 0.01);
    }
    
  //Test if it can find the right gap with pre-planned longitudinal maneuver at the end
  
  
  // Test if it can find the right gap with pre-planned complex maneuver  
/*  @Test
  public void testFindTrajectoryGapsWithComplexManeuver() {
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
  public void testManeuverGeneration() {
    List<Double> speeds = new ArrayList<>();
    speeds.add(5.0);

    Route route = generateRouteWithSpeedLimits(speeds, 5.0);


    routeService.processRoute(route);
    Trajectory traj = new Trajectory(0.0, 10.0);

    cruise.planTrajectory(traj, 0.0);
  }*/
}