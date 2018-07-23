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

package gov.dot.fhwa.saxton.carma.guidance.routefollowing;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import gov.dot.fhwa.saxton.carma.guidance.ArbitratorService;
import gov.dot.fhwa.saxton.carma.guidance.IGuidanceCommands;
import gov.dot.fhwa.saxton.carma.guidance.ManeuverPlanner;
import gov.dot.fhwa.saxton.carma.guidance.Tracking;
import gov.dot.fhwa.saxton.carma.guidance.conflictdetector.IConflictDetector;
import gov.dot.fhwa.saxton.carma.guidance.lightbar.ILightBarManager;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.IComplexManeuver;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.IManeuverInputs;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.ISimpleManeuver;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.LateralManeuver;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.LongitudinalManeuver;
import gov.dot.fhwa.saxton.carma.guidance.mobilityrouter.IMobilityRouter;
import gov.dot.fhwa.saxton.carma.guidance.params.ParameterSource;
import gov.dot.fhwa.saxton.carma.guidance.plugins.ITacticalPlugin;
import gov.dot.fhwa.saxton.carma.guidance.plugins.PluginManagementService;
import gov.dot.fhwa.saxton.carma.guidance.plugins.PluginServiceLocator;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.IPubSubService;
import gov.dot.fhwa.saxton.carma.guidance.trajectory.Trajectory;
import gov.dot.fhwa.saxton.carma.guidance.util.GuidanceRouteService;
import gov.dot.fhwa.saxton.carma.guidance.util.ILogger;
import gov.dot.fhwa.saxton.carma.guidance.util.ILoggerFactory;
import gov.dot.fhwa.saxton.carma.guidance.util.LoggerManager;
import gov.dot.fhwa.saxton.carma.guidance.util.RequiredLane;
import gov.dot.fhwa.saxton.carma.guidance.util.SpeedLimit;
import gov.dot.fhwa.saxton.carma.guidance.util.trajectoryconverter.ITrajectoryConverter;

import static org.junit.Assert.*;
import static org.mockito.Mockito.*;

import java.util.SortedSet;
import java.util.TreeSet;

public class RouteFollowingPluginTest {
    private GuidanceRouteService routeService;
    private RouteFollowingPlugin rfp;
    private Trajectory traj;
    private static final double TRAJ_START = 0.0;
    private static final double TRAJ_END = 1000.0;

    @Test
    public void testFullOverlap() {
        IComplexManeuver overlappingMvr = mock(IComplexManeuver.class);
        when(overlappingMvr.getStartDistance()).thenReturn(TRAJ_START);
        when(overlappingMvr.getEndDistance()).thenReturn(TRAJ_END);
        traj.setComplexManeuver(overlappingMvr);

        boolean noErrors = true;

        try {
            rfp.planTrajectory(traj, 0.0);
        } catch (Exception e) {
            System.out.println(e.getMessage());
            e.printStackTrace();
            noErrors = false;
        }

        assertTrue(noErrors);
    }

    @Test
    public void testPartialOverlapMiddle() {
        LongitudinalManeuver overlappingMvr = mock(LongitudinalManeuver.class);
        when(overlappingMvr.getStartDistance()).thenReturn(600.0);
        when(overlappingMvr.getEndDistance()).thenReturn(650.0);
        traj.addManeuver(overlappingMvr);

        boolean noErrors = true;

        try {
            rfp.planTrajectory(traj, 0.0);
        } catch (Exception e) {
            System.out.println(e.getMessage());
            e.printStackTrace();
            noErrors = false;
        }

        assertTrue(noErrors);
    }

    @Test
    public void testPartialOverlapStart() {
        LongitudinalManeuver overlappingMvr = mock(LongitudinalManeuver.class);
        when(overlappingMvr.getStartDistance()).thenReturn(400.0);
        when(overlappingMvr.getEndDistance()).thenReturn(550.0);
        traj.addManeuver(overlappingMvr);

        boolean noErrors = true;

        try {
            rfp.planTrajectory(traj, 0.0);
        } catch (Exception e) {
            System.out.println(e.getMessage());
            e.printStackTrace();
            noErrors = false;
        }

        assertTrue(noErrors);
    }

    @Test
    public void testPartialOverlapEnd() {
        LongitudinalManeuver overlappingMvr = mock(LongitudinalManeuver.class);
        when(overlappingMvr.getStartDistance()).thenReturn(600.0);
        when(overlappingMvr.getEndDistance()).thenReturn(750.0);
        traj.addManeuver(overlappingMvr);

        boolean noErrors = true;

        try {
            rfp.planTrajectory(traj, 0.0);
        } catch (Exception e) {
            System.out.println(e.getMessage());
            e.printStackTrace();
            noErrors = false;
        }

        assertTrue(noErrors);
    }


    @Before
    public void setUp() {
        ILoggerFactory mockFact = mock(ILoggerFactory.class);
        ILogger mockLogger = mock(ILogger.class);
        when(mockFact.createLoggerForClass(any())).thenReturn(mockLogger);
        LoggerManager.setLoggerFactory(mockFact);

        ParameterSource mockParameterSource = mock(ParameterSource.class);
        when(mockParameterSource.getDouble("~lane_change_rate_factor")).thenReturn(0.75);
        when(mockParameterSource.getDouble("~lane_change_delay_factor")).thenReturn(1.5);
        when(mockParameterSource.getDouble("~lane_change_safety_factor")).thenReturn(1.5);

        routeService = mock(GuidanceRouteService.class);
        SortedSet<RequiredLane> requiredLanes = new TreeSet<>((a, b) -> Double.compare(a.getLocation(), b.getLocation()));
        requiredLanes.add(new RequiredLane(700.0, 1));
        when(routeService.getRequiredLanesInRange(anyDouble(), anyDouble())).thenReturn(requiredLanes);
        when(routeService.getSpeedLimitAtLocation(anyDouble())).thenReturn(new SpeedLimit(1000.0, 45.0));

        PluginManagementService mockPluginManagementService = mock(PluginManagementService.class);
        when(mockPluginManagementService.getTacticalPluginByName(anyString())).thenReturn(mock(ITacticalPlugin.class));

        PluginServiceLocator psl = new PluginServiceLocator(mock(ArbitratorService.class),
                mockPluginManagementService, mock(IPubSubService.class), mockParameterSource,
                new ManeuverPlanner(mock(IGuidanceCommands.class), mock(IManeuverInputs.class)), routeService,
                mock(IMobilityRouter.class), mock(IConflictDetector.class), mock(ITrajectoryConverter.class),
                mock(ILightBarManager.class), mock(Tracking.class));
        rfp = new RouteFollowingPlugin(psl);
        traj = new Trajectory(TRAJ_START, TRAJ_END); 

        rfp.onResume();
    }
}