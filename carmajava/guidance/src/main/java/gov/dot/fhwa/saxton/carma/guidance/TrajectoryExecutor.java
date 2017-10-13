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

//TODO: Naming convention of "package gov.dot.fhwa.saxton.carmajava.<template>;"
//Originally "com.github.rosjava.carmajava.template;"
package gov.dot.fhwa.saxton.carma.guidance;

import cav_msgs.RouteState;
import cav_msgs.SystemAlert;
import cav_srvs.GetDriversWithCapabilities;
import cav_srvs.GetDriversWithCapabilitiesRequest;
import cav_srvs.GetDriversWithCapabilitiesResponse;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.*;
import gov.dot.fhwa.saxton.carma.guidance.trajectory.IManeuver;
import gov.dot.fhwa.saxton.carma.guidance.trajectory.OnTrajectoryProgressCallback;
import gov.dot.fhwa.saxton.carma.guidance.trajectory.Trajectory;
import gov.dot.fhwa.saxton.carma.guidance.trajectory.TrajectoryExecutorWorker;

import org.apache.commons.logging.Log;
import org.ros.node.ConnectedNode;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

/**
 * Guidance package TrajectoryExecutor component
 * <p>
 * Guidance component responsible for performing the timely execution of planned
 * maneuvers in a Trajectory planned by the Arbitrator and the Guidance package's
 * currently configured plugins.
 */
public class TrajectoryExecutor extends GuidanceComponent {
    // Member variables
    protected ISubscriber<RouteState> routeStateSubscriber;
    protected GuidanceCommands commands;
    protected AtomicReference<GuidanceState> state;
    protected TrajectoryExecutorWorker trajectoryExecutorWorker;

    protected long startTime = 0;
    protected long holdTimeMs = 0;
    protected double operatingSpeed;
    protected double amplitude;
    protected double phase;
    protected double period;
    protected double maxAccel;
    protected final long sleepDurationMillis = 100;

    public TrajectoryExecutor(AtomicReference<GuidanceState> state, IPubSubService iPubSubService, ConnectedNode node,
            GuidanceCommands commands) {
        super(state, iPubSubService, node);
        this.state = state;
        this.commands = commands;
    }

    @Override
    public String getComponentName() {
        return "Guidance.TrajectoryExecutor";
    }

    @Override
    public void onGuidanceStartup() {
        routeStateSubscriber = pubSubService.getSubscriberForTopic("route_status", RouteState._TYPE);
        routeStateSubscriber.registerOnMessageCallback(new OnMessageCallback<RouteState>() {
            @Override
            public void onMessage(RouteState msg) {
                log.info("Received RouteState:" + msg);
                trajectoryExecutorWorker.updateDowntrackDistance(msg.getDownTrack());
            }
        });

        operatingSpeed = node.getParameterTree().getDouble("~trajectory_operating_speed");
        amplitude = node.getParameterTree().getDouble("~trajectory_amplitude");
        phase = node.getParameterTree().getDouble("~trajectory_phase");
        period = node.getParameterTree().getDouble("~trajectory_period");
        maxAccel = node.getParameterTree().getDouble("~max_acceleration_capability");
        holdTimeMs = (long) (node.getParameterTree().getDouble("~trajectory_initial_hold_duration") * 1000);
    }

    @Override
    public void onSystemReady() {
        // NO-OP
    }

    @Override
    public void onGuidanceEnable() {
        startTime = System.currentTimeMillis();
    }

    /**
     * Compute the sinusoidal part of the trajectory
     * 
     * @param t The current time in milliseconds
     * @param amplitude the max/min of the sinusoidal curve in m/s
     * @param period The number of seconds to complete a cycle
     * @param phase Where in the cycle to start
     * 
     * @return The current value of the sinusoidal trajectory component
     */
    private double computeSin(double t, double amplitude, double period, double phase) {
        double s = t / 1000.0;
        double pFactor = 2 * Math.PI / period;

        return amplitude * Math.sin((pFactor * s) + phase);
    }

    public void loop() {
        try {
            // Generate a simple sin(t) speed command
            if (state.get() == GuidanceState.ENGAGED) {
                if (System.currentTimeMillis() - startTime < holdTimeMs) {
                    commands.setCommand(operatingSpeed, maxAccel);
                } else {
                    commands.setCommand(
                            operatingSpeed + computeSin(System.currentTimeMillis(), amplitude, period, phase),
                            maxAccel);
                }
            }

            Thread.sleep(sleepDurationMillis);
        } catch (InterruptedException e) {
        }
    }

    public void abortTrajectory() {
        trajectoryExecutorWorker.abortTrajectory();
    }

    public IManeuver getCurrentLateralManeuver() {
        return trajectoryExecutorWorker.getCurrentLateralManeuver();        
    }

    public IManeuver getCurrentLongitudinalManeuver() {
        return trajectoryExecutorWorker.getCurrentLongitudinalManeuver();        
    }

    public IManeuver getNextLateralManeuver() {
        return trajectoryExecutorWorker.getNextLateralManeuver();
    }

    public IManeuver getNextLongitudinalManeuver() {
        return trajectoryExecutorWorker.getNextLongitudinalManeuver();
    }

    public double getTrajectoryCompletionPct() {
        return trajectoryExecutorWorker.getTrajectoryCompletionPct();
    }

    public void registerOnTrajectoryProgressCallback(double pct, OnTrajectoryProgressCallback callback) {
        trajectoryExecutorWorker.registerOnTrajectoryProgressCallback(pct, callback);
    }

    public void runTrajectory(Trajectory traj) {
        trajectoryExecutorWorker.runTrajectory(traj);
    }
}
