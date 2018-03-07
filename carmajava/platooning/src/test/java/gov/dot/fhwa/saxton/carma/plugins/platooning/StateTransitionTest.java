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

package gov.dot.fhwa.saxton.carma.plugins.platooning;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.Mockito.*;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import gov.dot.fhwa.saxton.carma.guidance.ArbitratorService;
import gov.dot.fhwa.saxton.carma.guidance.IGuidanceCommands;
import gov.dot.fhwa.saxton.carma.guidance.ManeuverPlanner;
import gov.dot.fhwa.saxton.carma.guidance.arbitrator.TrajectoryPlanningResponse;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.AccStrategyManager;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.IManeuverInputs;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.NoOpAccStrategyFactory;
import gov.dot.fhwa.saxton.carma.guidance.params.ParameterSource;
import gov.dot.fhwa.saxton.carma.guidance.plugins.PluginManagementService;
import gov.dot.fhwa.saxton.carma.guidance.plugins.PluginServiceLocator;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.IPubSubService;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.IPublisher;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.ISubscriber;
import gov.dot.fhwa.saxton.carma.guidance.trajectory.Trajectory;
import gov.dot.fhwa.saxton.carma.guidance.util.GuidanceRouteService;
import gov.dot.fhwa.saxton.carma.guidance.util.ILogger;
import gov.dot.fhwa.saxton.carma.guidance.util.ILoggerFactory;
import gov.dot.fhwa.saxton.carma.guidance.util.LoggerManager;
import gov.dot.fhwa.saxton.carma.guidance.util.SpeedLimit;

public class StateTransitionTest {

    private PlatooningPlugin platooning;
    private GuidanceRouteService routeService;
    private ILogger mockLogger;
    private PluginServiceLocator psl;
    private IManeuverInputs input;
    
    @Before
    public void setup() {
        ILoggerFactory mockFact = mock(ILoggerFactory.class);
        mockLogger = mock(ILogger.class);
        when(mockFact.createLoggerForClass(any())).thenReturn(mockLogger);
        LoggerManager.setLoggerFactory(mockFact);
        
        NoOpAccStrategyFactory noOpAccStrategyFactory = new NoOpAccStrategyFactory();
        AccStrategyManager.setAccStrategyFactory(noOpAccStrategyFactory);
        
        input = mock(IManeuverInputs.class);
        
        routeService = mock(GuidanceRouteService.class);
        psl = new PluginServiceLocator(mock(ArbitratorService.class),
                mock(PluginManagementService.class), mock(IPubSubService.class), mock(ParameterSource.class),
                new ManeuverPlanner(mock(IGuidanceCommands.class), input), routeService);
        platooning = new PlatooningPlugin(psl);
        platooning.onInitialize();
        platooning.mobilityIntroPublisher = mock(IPublisher.class);
        platooning.newPlanSub = mock(ISubscriber.class);
        platooning.cmdSpeedSub = mock(ISubscriber.class);
        platooning.onResume();
    }
    
    @After
    public void onFinish() {
        platooning.onSuspend();
        platooning.onTerminate();
    }
    
    @Test
    public void initialStateTest() {
        assertTrue(platooning.state instanceof StandbyState);
    }
    
    @Test
    public void standbyToLeader() {
        // Standby state to Leader state when platooning is enabled in the next Trajectory
        Trajectory traj = new Trajectory(0.0, 50.0);
        when(routeService.isAlgorithmEnabledInRange(0.0, 50.0, platooning.PLATOONING_FLAG)).thenReturn(true);
        TrajectoryPlanningResponse tpr = platooning.planTrajectory(traj, 0.0);
        assertTrue(platooning.state instanceof LeaderState);
        assertEquals(1, tpr.getRequests().size());
        assertEquals(100, tpr.getProposedReplanDelay().get().longValue());
    }
    
    @Test
    public synchronized void leaderToFollower() {
        // Leader state to Follower state when we have other platooning vehicle in front of us
        // TODO This state transition need to happened after the leader from front platoon agreed our JOIN_A_PLATOON message
        IPlatooningState leaderState = new LeaderState(platooning, mockLogger, psl); 
        platooning.setState(leaderState);
        platooning.state.checkCurrentState();
        platooning.platoonManager.platoon.add(new PlatoonMember("", 0.0, 0.0, 0.0, Long.MAX_VALUE));
        platooning.state.checkCurrentState();
        assertTrue(platooning.state instanceof FollowerState);
    }
    
    @Test
    public void leaderToStandby() {
        // Leader state to Standby state when the platooning algorithm is disanbled in the next trajectory
        IPlatooningState leaderState = new LeaderState(platooning, mockLogger, psl); 
        platooning.setState(leaderState);
        Trajectory traj = new Trajectory(25.0, 50.0);
        when(routeService.isAlgorithmEnabledInRange(25.0, 50.0, platooning.PLATOONING_FLAG)).thenReturn(false);
        when(routeService.getCurrentDowntrackDistance()).thenReturn(20.0);
        when(input.getCurrentSpeed()).thenReturn(5.0);
        when(routeService.getSpeedLimitAtLocation(25.0)).thenReturn(new SpeedLimit(25.0, 5.0));
        TrajectoryPlanningResponse response = platooning.planTrajectory(traj, 25.0);
        assertEquals(0, response.getRequests().size());
        platooning.state.checkCurrentState();
        assertTrue(platooning.state instanceof LeaderState);
        try {
            Thread.sleep(5000);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
        platooning.state.checkCurrentState();
        assertTrue(platooning.state instanceof StandbyState);
    }
    
    @Test
    public synchronized void followerToStandbyCase() {
        // Follower state to Standby state when the platooning algorithm is disanbled in the next trajectory
        IPlatooningState followerState = new FollowerState(platooning, mockLogger, psl);
        platooning.setState(followerState);
        Trajectory traj = new Trajectory(25.0, 50.0);
        when(routeService.isAlgorithmEnabledInRange(25.0, 50.0, platooning.PLATOONING_FLAG)).thenReturn(false);
        when(routeService.getCurrentDowntrackDistance()).thenReturn(20.0);
        when(input.getCurrentSpeed()).thenReturn(5.0);
        when(routeService.getSpeedLimitAtLocation(25.0)).thenReturn(new SpeedLimit(25.0, 5.0));
        platooning.platoonManager.platoon.add(new PlatoonMember("", 0.0, 0.0, 0.0, Long.MAX_VALUE));
        TrajectoryPlanningResponse response = platooning.planTrajectory(traj, 25.0);
        assertEquals(0, response.getRequests().size());
        platooning.state.checkCurrentState();
        assertTrue(platooning.state instanceof FollowerState);
        try {
            Thread.sleep(5000);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
        platooning.state.checkCurrentState();
        assertTrue(platooning.state instanceof StandbyState);
    }
    
    @Test
    public void followerToLeader() {
        // Follower state to Leader state when the host vehicle is the first one in the platooning
        IPlatooningState followerState = new FollowerState(platooning, mockLogger, psl);
        platooning.setState(followerState);
        platooning.state.checkCurrentState();
        assertTrue(platooning.state instanceof LeaderState);
    }
}
