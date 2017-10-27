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

package gov.dot.fhwa.saxton.carma.guidance;

import cav_msgs.RouteState;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.IManeuver;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.LongitudinalManeuver;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.*;
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
    protected Trajectory currentTrajectory;
    protected boolean bufferedTrajectoryRunning = false;

    protected boolean useSinTrajectory = false;
    protected long startTime = 0;
    protected long holdTimeMs = 0;
    protected double operatingSpeed;
    protected double amplitude;
    protected double phase;
    protected double period;
    protected double maxAccel;
    protected long sleepDurationMillis = 100;

    public TrajectoryExecutor(AtomicReference<GuidanceState> state, IPubSubService iPubSubService, ConnectedNode node,
            GuidanceCommands commands) {
        super(state, iPubSubService, node);
        this.state = state;
        this.commands = commands;

        double maneuverTickFreq = node.getParameterTree().getDouble("~maneuver_tick_freq", 10.0);

        trajectoryExecutorWorker = new TrajectoryExecutorWorker(commands, maneuverTickFreq, node.getLog());
    }

    @Override
    public String getComponentName() {
        return "Guidance.TrajectoryExecutor";
    }

    @Override
    public void onGuidanceStartup() {
        routeStateSubscriber = pubSubService.getSubscriberForTopic("route_state", RouteState._TYPE);
        routeStateSubscriber.registerOnMessageCallback(new OnMessageCallback<RouteState>() {
            @Override
            public void onMessage(RouteState msg) {
                log.info("Received RouteState. New downtrack distance: " + msg.getDownTrack());
                trajectoryExecutorWorker.updateDowntrackDistance(msg.getDownTrack());
            }
        });

        operatingSpeed = node.getParameterTree().getDouble("~trajectory_operating_speed");
        amplitude = node.getParameterTree().getDouble("~trajectory_amplitude");
        phase = node.getParameterTree().getDouble("~trajectory_phase");
        period = node.getParameterTree().getDouble("~trajectory_period");
        maxAccel = node.getParameterTree().getDouble("~max_acceleration_capability");
        holdTimeMs = (long) (node.getParameterTree().getDouble("~trajectory_initial_hold_duration") * 1000);
        useSinTrajectory = node.getParameterTree().getBoolean("~use_sin_trajectory", false);
        sleepDurationMillis = (long) (1000.0 / node.getParameterTree().getDouble("~trajectory_executor_frequency"));
    }

    @Override
    public void onSystemReady() {
        // NO-OP
    }

    @Override
    public void onGuidanceEnable() {
        startTime = (long) node.getCurrentTime().toSeconds() * 1000;

        if (currentTrajectory != null && !bufferedTrajectoryRunning) {
            log.info("Running buffered trajectory!");
            trajectoryExecutorWorker.runTrajectory(currentTrajectory);
        }
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

    @Override
    public void loop() throws InterruptedException {
        // Generate a simple sin(t) speed command
        if (state.get() == GuidanceState.ENGAGED && useSinTrajectory) {
            if ((node.getCurrentTime().toSeconds() * 1000) - startTime < holdTimeMs) {
                commands.setCommand(operatingSpeed, maxAccel);
            } else {
                commands.setCommand(operatingSpeed + computeSin(System.currentTimeMillis(), amplitude, period, phase),
                        maxAccel);
            }
        }

        Thread.sleep(sleepDurationMillis);
    }

  /**
   * Abort the current and queued trajectories and the currently executing maneuvers
   */
    public void abortTrajectory() {
        log.info("Trajectory executor commanded to abort trajectory");
        trajectoryExecutorWorker.abortTrajectory();
    }

  /**
   * Get the current lateral maneuver, null if none are currently executing
   */
    public IManeuver getCurrentLateralManeuver() {
        return trajectoryExecutorWorker.getCurrentLateralManeuver();        
    }

  /**
   * Get the current longitudinal maneuver, null if none are currently executing
   */
    public IManeuver getCurrentLongitudinalManeuver() {
        return trajectoryExecutorWorker.getCurrentLongitudinalManeuver();        
    }

  /**
   * Get the next lateral maneuver, null if none are currently executing
   */
    public IManeuver getNextLateralManeuver() {
        return trajectoryExecutorWorker.getNextLateralManeuver();
    }

  /**
   * Get the next longitudinal maneuver, null if none are currently executing
   */
    public IManeuver getNextLongitudinalManeuver() {
        return trajectoryExecutorWorker.getNextLongitudinalManeuver();
    }

  /**
   * Get the current complection pct of the trajectory, -1.0 if a trajectory isn't currently executing
   */
    public double getTrajectoryCompletionPct() {
        return trajectoryExecutorWorker.getTrajectoryCompletionPct();
    }

  /**
   * Register a callback function to be invoked when the specified percent completion is acheived
   */
    public void registerOnTrajectoryProgressCallback(double pct, OnTrajectoryProgressCallback callback) {
        trajectoryExecutorWorker.registerOnTrajectoryProgressCallback(pct, callback);
    }

  /**
   * Submit the specified trajectory for execution
   * </p>
   * If no trajectories are running it will be run right away, otherwise it will be queued to run after the current
   * trajectory finishes execution.
   */
    public void runTrajectory(Trajectory traj) {
        log.info("TrajectoryExecutor received new trajectory!");
        int idx = 1;
        for (IManeuver m : traj.getManeuvers()) {
            log.info("Maneuver #" + idx + " from [" + m.getStartDistance() + ", " + m.getEndDistance() + ") of type" + (m instanceof LongitudinalManeuver ? "LONGITUDINAL" : "LATERAL"));
            if (m instanceof LongitudinalManeuver) {
                log.info("Speeds from " + m.getStartSpeed() + " to " + m.getTargetSpeed());
            }
            idx++;
        }

        if (state.get() == GuidanceState.ENGAGED) {
            log.info("TrajectoryExecutor running trajectory!");
            trajectoryExecutorWorker.runTrajectory(traj);
            bufferedTrajectoryRunning = true;
        } else {
            currentTrajectory = traj;
        }
    }
}
