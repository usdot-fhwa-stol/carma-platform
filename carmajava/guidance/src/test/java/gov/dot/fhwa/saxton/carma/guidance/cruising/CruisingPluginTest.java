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
import java.util.SortedSet;
import java.util.TreeSet;

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
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.FutureLateralManeuver;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.FutureLongitudinalManeuver;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.IManeuver;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.IManeuverInputs;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.NoOpAccStrategyFactory;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.SlowDown;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.SpeedUp;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.SteadySpeed;
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
import gov.dot.fhwa.saxton.carma.plugins.speedharm.SpeedHarmonizationManeuver;

public class CruisingPluginTest {

    //private NodeConfiguration nodeConfig = NodeConfiguration.newPrivate();
    //private MessageFactory messageFactory = nodeConfig.getTopicMessageFactory();
    private GuidanceRouteService routeService;
    private CruisingPlugin cruise;

    @Before
    public void setup() {
        
        ILoggerFactory mockFact = mock(ILoggerFactory.class);
        ILogger mockLogger = mock(ILogger.class);
        when(mockFact.createLoggerForClass(any())).thenReturn(mockLogger);
        LoggerManager.setLoggerFactory(mockFact);
        
        NoOpAccStrategyFactory noOpAccStrategyFactory = new NoOpAccStrategyFactory();
        AccStrategyManager.setAccStrategyFactory(noOpAccStrategyFactory);
        
        routeService = mock(GuidanceRouteService.class);
        PluginServiceLocator psl = new PluginServiceLocator(mock(ArbitratorService.class),
                mock(PluginManagementService.class), mock(IPubSubService.class), mock(ParameterSource.class),
                new ManeuverPlanner(mock(IGuidanceCommands.class), mock(IManeuverInputs.class)), routeService);
        cruise = new CruisingPlugin(psl);
        cruise.onInitialize();
        cruise.maxAccel_ = 2.5;
    }

    //Test if CP can find the right longitudinal gap in empty trajectory
    @Test
    public void testFindTrajectoryGapsWithEmptyTrajectory() {
        Trajectory t = new Trajectory(0.0, 50.0);
        List<TrajectorySegment> gaps = cruise.findTrajectoryGaps(t, 3.0);
        assertEquals(1, gaps.size());
        assertEquals(0.0, gaps.get(0).startLocation, 0.01);
        assertEquals(50.0, gaps.get(0).endLocation, 0.01);
        assertEquals(3.0, gaps.get(0).startSpeed, 0.01);
    }
    
    //Test if CP can find the right longitudinal gap in a full trajectory
    @Test
    public void testFindTrajectoryGapsWithFullTrajectory() {
        Trajectory t = new Trajectory(0.0, 5.0);
        SteadySpeed mockLonManeuver = mock(SteadySpeed.class);
        when(mockLonManeuver.getStartDistance()).thenReturn(0.0);
        when(mockLonManeuver.getEndDistance()).thenReturn(5.0);
        when(mockLonManeuver.getTargetSpeed()).thenReturn(1.0);
        t.addManeuver(mockLonManeuver);
        List<TrajectorySegment> gaps = cruise.findTrajectoryGaps(t, 1.0);
        assertEquals(0, gaps.size());
    }
    
    //Test if it can find the right gap with pre-planned longitudinal maneuvers
    @Test
    public void testFindTrajectoryGapsWithOnlyLonManeuvers() {
        Trajectory t = new Trajectory(0.0, 100.0);
        SpeedUp mockSpeedUpManeuver = mock(SpeedUp.class);
        SteadySpeed mockSteadyManeuver = mock(SteadySpeed.class);
        SlowDown mockSlowDownManeuver = mock(SlowDown.class);
        when(mockSpeedUpManeuver.getStartDistance()).thenReturn(5.0);
        when(mockSpeedUpManeuver.getEndDistance()).thenReturn(20.0);
        when(mockSpeedUpManeuver.getTargetSpeed()).thenReturn(1.5);
        when(mockSteadyManeuver.getStartDistance()).thenReturn(40.0);
        when(mockSteadyManeuver.getEndDistance()).thenReturn(60.0);
        when(mockSteadyManeuver.getTargetSpeed()).thenReturn(2.5);
        when(mockSlowDownManeuver.getStartDistance()).thenReturn(60.0);
        when(mockSlowDownManeuver.getEndDistance()).thenReturn(85.0);
        when(mockSlowDownManeuver.getTargetSpeed()).thenReturn(2.0);
        t.addManeuver(mockSpeedUpManeuver);
        t.addManeuver(mockSteadyManeuver);
        t.addManeuver(mockSlowDownManeuver);
        List<TrajectorySegment> gaps = cruise.findTrajectoryGaps(t, 1.2);
        assertEquals(3, gaps.size());
        assertEquals(0.0, gaps.get(0).startLocation, 0.01);
        assertEquals(5.0, gaps.get(0).endLocation, 0.01);
        assertEquals(1.2, gaps.get(0).startSpeed, 0.01);
        assertEquals(20.0, gaps.get(1).startLocation, 0.01);
        assertEquals(40.0, gaps.get(1).endLocation, 0.01);
        assertEquals(1.5, gaps.get(1).startSpeed, 0.01);
        assertEquals(85, gaps.get(2).startLocation, 0.01);
        assertEquals(100, gaps.get(2).endLocation, 0.01);
        assertEquals(2.0, gaps.get(2).startSpeed, 0.01);
    }
    
    //Test if it can find the right gap with pre-planned complex maneuver
    @Test
    public void testFindTrajectoryGapsOnlyComplexManeuver() {
        Trajectory t = new Trajectory(0.0, 100.0);
        SpeedHarmonizationManeuver mockshm = mock(SpeedHarmonizationManeuver.class);
        when(mockshm.getStartDistance()).thenReturn(40.0);
        t.setComplexManeuver(mockshm);
        List<TrajectorySegment> gaps = cruise.findTrajectoryGaps(t, 1.0);
        assertEquals(1, gaps.size());
        assertEquals(0.0, gaps.get(0).startLocation, 0.01);
        assertEquals(40.0, gaps.get(0).endLocation, 0.01);
        assertEquals(1.0, gaps.get(0).startSpeed, 0.01);
    }
    
    //Test if it can find the right gap with pre-planned future lat/lon maneuvers
    @Test
    public void testFindTrajectoryGapsOnlyFutureManeuver() {
        Trajectory t = new Trajectory(0.0, 50.0);
        FutureLateralManeuver fLatM = mock(FutureLateralManeuver.class);
        FutureLongitudinalManeuver fLonM = mock(FutureLongitudinalManeuver.class);
        when(fLatM.getStartDistance()).thenReturn(20.0);
        when(fLatM.getEndDistance()).thenReturn(30.0);
        when(fLonM.getStartDistance()).thenReturn(20.0);
        when(fLonM.getEndDistance()).thenReturn(30.0);
        when(fLonM.getTargetSpeed()).thenReturn(2.0);
        t.addManeuver(fLatM);
        t.addManeuver(fLonM);
        List<TrajectorySegment> gaps = cruise.findTrajectoryGaps(t, 1.0);
        assertEquals(2, gaps.size());
        assertEquals(0.0, gaps.get(0).startLocation, 0.01);
        assertEquals(20.0, gaps.get(0).endLocation, 0.01);
        assertEquals(1.0, gaps.get(0).startSpeed, 0.01);
        assertEquals(30.0, gaps.get(1).startLocation, 0.01);
        assertEquals(50.0, gaps.get(1).endLocation, 0.01);
        assertEquals(2.0, gaps.get(1).startSpeed, 0.01);
    }
    
    //Test if cruising plugin can actually fill a trajectory with one maneuver at the middle
    //TODO can not run this test successfully, need help it throw NPE at line 189
    @Test
    public void testManeuverGeneration() {
        Trajectory t = new Trajectory(0.0, 67.0);
        SteadySpeed mockSteadyManeuver = mock(SteadySpeed.class);
        when(mockSteadyManeuver.getStartDistance()).thenReturn(30.0);
        when(mockSteadyManeuver.getEndDistance()).thenReturn(50.0);
        when(mockSteadyManeuver.getTargetSpeed()).thenReturn(5.0);
        t.addManeuver(mockSteadyManeuver);
        
        SortedSet<SpeedLimit> trajLimitsAtRange1 = new TreeSet<>((a, b) -> Double.compare(a.getLocation(), b.getLocation()));
        trajLimitsAtRange1.add(new SpeedLimit(15, 5));
        SpeedLimit trajLimitAtLocation1 = new SpeedLimit(30, 5);
        when(routeService.getSpeedLimitsInRange(0.0, 30.0)).thenReturn(trajLimitsAtRange1);
        when(routeService.getSpeedLimitAtLocation(30.0)).thenReturn(trajLimitAtLocation1);
        SortedSet<SpeedLimit> trajLimitsAtRange2 = new TreeSet<>((a, b) -> Double.compare(a.getLocation(), b.getLocation()));
        SpeedLimit trajLimitAtLocation2 = new SpeedLimit(70, 10);
        when(routeService.getSpeedLimitsInRange(50.0, 67.0)).thenReturn(trajLimitsAtRange2);
        when(routeService.getSpeedLimitAtLocation(67.0)).thenReturn(trajLimitAtLocation2);
        cruise.planTrajectory(t, 0.0);
        assertEquals(4, t.getLongitudinalManeuvers().size());
        assertEquals(0, t.getLongitudinalManeuvers().get(0).getStartDistance(), 0.01);
        assertEquals(6, t.getLongitudinalManeuvers().get(0).getEndDistance(), 0.01);
        assertEquals(0, t.getLongitudinalManeuvers().get(0).getStartSpeed(), 0.01);
        assertEquals(5, t.getLongitudinalManeuvers().get(0).getTargetSpeed(), 0.01);
        assertEquals(6, t.getLongitudinalManeuvers().get(1).getStartDistance(), 0.01);
        assertEquals(30, t.getLongitudinalManeuvers().get(1).getEndDistance(), 0.01);
        assertEquals(5, t.getLongitudinalManeuvers().get(1).getStartSpeed(), 0.01);
        assertEquals(5, t.getLongitudinalManeuvers().get(1).getTargetSpeed(), 0.01);
        assertEquals(50, t.getLongitudinalManeuvers().get(3).getStartDistance(), 0.01);
        assertEquals(67, t.getLongitudinalManeuvers().get(3).getEndDistance(), 0.01);
        assertEquals(5, t.getLongitudinalManeuvers().get(3).getStartSpeed(), 0.01);
        assertEquals(10, t.getLongitudinalManeuvers().get(3).getTargetSpeed(), 0.01);
    }
}