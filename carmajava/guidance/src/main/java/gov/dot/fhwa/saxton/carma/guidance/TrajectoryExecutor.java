/*
 * Copyright (C) 2017 LEIDOS.
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
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.IComplexManeuver;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.IManeuver;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.LongitudinalManeuver;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.*;
import gov.dot.fhwa.saxton.carma.guidance.trajectory.OnTrajectoryProgressCallback;
import gov.dot.fhwa.saxton.carma.guidance.trajectory.Trajectory;
import gov.dot.fhwa.saxton.carma.guidance.trajectory.TrajectoryExecutorWorker;

import org.ros.exception.RosRuntimeException;
import org.ros.node.ConnectedNode;

/**
 * Guidance package TrajectoryExecutor component
 * <p>
 * Guidance component responsible for performing the timely execution of planned
 * maneuvers in a Trajectory planned by the Arbitrator and the Guidance package's
 * currently configured plugins.
 */
public class TrajectoryExecutor extends GuidanceComponent implements IStateChangeListener {
    // Member variables
    protected ISubscriber<RouteState> routeStateSubscriber;
    protected GuidanceCommands commands;
    protected TrajectoryExecutorWorker trajectoryExecutorWorker;
    protected Trajectory currentTrajectory;
    protected Tracking tracking_;
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

    public TrajectoryExecutor(GuidanceStateMachine stateMachine, IPubSubService iPubSubService, ConnectedNode node,
            GuidanceCommands commands, Tracking tracking) {
        super(stateMachine, iPubSubService, node);
        this.commands = commands;
        this.tracking_ = tracking;

        double maneuverTickFreq = node.getParameterTree().getDouble("~maneuver_tick_freq", 10.0);
        trajectoryExecutorWorker = new TrajectoryExecutorWorker(commands, maneuverTickFreq);
        
        jobQueue.add(this::onStartup);
        stateMachine.registerStateChangeListener(this);
    }

    @Override
    public String getComponentName() {
        return "Guidance.TrajectoryExecutor";
    }

    @Override
    public void onStartup() {
        operatingSpeed = node.getParameterTree().getDouble("~trajectory_operating_speed");
        amplitude = node.getParameterTree().getDouble("~trajectory_amplitude");
        phase = node.getParameterTree().getDouble("~trajectory_phase");
        period = node.getParameterTree().getDouble("~trajectory_period");
        maxAccel = node.getParameterTree().getDouble("~max_acceleration_capability");
        holdTimeMs = (long) (node.getParameterTree().getDouble("~trajectory_initial_hold_duration") * 1000);
        useSinTrajectory = node.getParameterTree().getBoolean("~use_sin_trajectory", false);
        sleepDurationMillis = (long) (1000.0 / node.getParameterTree().getDouble("~trajectory_executor_frequency"));

        routeStateSubscriber = pubSubService.getSubscriberForTopic("route_state", RouteState._TYPE);
        routeStateSubscriber.registerOnMessageCallback(new OnMessageCallback<RouteState>() {
            @Override
            public void onMessage(RouteState msg) {
                log.info("Received RouteState. New downtrack distance: " + msg.getDownTrack());
                trajectoryExecutorWorker.updateDowntrackDistance(msg.getDownTrack());
            }
        });

        currentState.set(GuidanceState.STARTUP);
    }

    @Override
    public void onSystemReady() {
        currentState.set(GuidanceState.DRIVERS_READY);
    }

    @Override
    public void onRouteActive() {
        currentState.set(GuidanceState.ACTIVE);
    }
    
    @Override
    public void onDeactivate() {
        currentState.set(GuidanceState.INACTIVE);
    }
    
    @Override
    public void onEngaged() {
        startTime = (long) node.getCurrentTime().toSeconds() * 1000;
        if (currentTrajectory != null && !bufferedTrajectoryRunning) {
            log.info("Running buffered trajectory!");
            tracking_.addNewTrajectory(currentTrajectory);
            trajectoryExecutorWorker.runTrajectory(currentTrajectory);
            bufferedTrajectoryRunning = true;
        }
        currentState.set(GuidanceState.ENGAGED);
    }

    @Override
    public void onCleanRestart() {
        currentState.set(GuidanceState.DRIVERS_READY);
        TrajectoryExecutor.this.abortTrajectory();
        trajectoryExecutorWorker.cleanRestart();
        this.unregisterAllTrajectoryProgressCallback();
        currentTrajectory = null;
        bufferedTrajectoryRunning = false;
        startTime = 0;
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
    public void timingLoop() throws InterruptedException {
        // Generate a simple sin(t) speed command
        if (currentState.get() == GuidanceState.ENGAGED && useSinTrajectory) {
            if ((node.getCurrentTime().toSeconds() * 1000) - startTime < holdTimeMs) {
                commands.setSpeedCommand(operatingSpeed, maxAccel);
            } else {
                commands.setSpeedCommand(operatingSpeed + computeSin(System.currentTimeMillis(), amplitude, period, phase),
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
   * Get the current complex maneuver, null if none are currently executing
   */
    public IManeuver getCurrentComplexManeuver() {
        return trajectoryExecutorWorker.getCurrentComplexManeuver();
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
   * Get the next complex maneuver, null if none are currently executing
   */
    public IManeuver getNextComplexManeuver() {
        return trajectoryExecutorWorker.getNextComplexManeuver();
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
     * Unregister the callback, ensuring it is no longer invoked at any point in time
     */
    public void unregisterOnTrajectoryProgressCallback(OnTrajectoryProgressCallback callback) {
        trajectoryExecutorWorker.unregisterOnTrajectoryProgressCallback(callback);
    }

    /**
     * Unregister all callbacks
     */
    public void unregisterAllTrajectoryProgressCallback() {
        trajectoryExecutorWorker.unregisterAllTrajectoryProgressCallback();
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
            String maneuverType = "LATERAL";
            if (m instanceof LongitudinalManeuver) {
                maneuverType = "LONGITUDINAL";
            } else if (m instanceof IComplexManeuver) {
                maneuverType = "COMPLEX";
            }

            log.info("Maneuver #" + idx + " from [" + m.getStartDistance() + ", " + m.getEndDistance() + ") of type " + maneuverType);
            if (m instanceof LongitudinalManeuver) {
                LongitudinalManeuver lonMvr = (LongitudinalManeuver) m;
                log.info("Speeds from " + lonMvr.getStartSpeed() + " to " + lonMvr.getTargetSpeed());
            }
            idx++;
        }

        if (currentState.get() == GuidanceState.ENGAGED) {
            log.info("TrajectoryExecutor running trajectory!");
            tracking_.addNewTrajectory(traj);
            trajectoryExecutorWorker.runTrajectory(traj);
            bufferedTrajectoryRunning = true;
        } else {
            currentTrajectory = traj;
        }
    }

    /**
     * Get the currently executing trajectory from the TrajectoryExecutor
     * 
     * @return The currently executing trajectory, or null if none is running
     */
    public Trajectory getCurrentTrajectory() {
        return trajectoryExecutorWorker.getCurrentTrajectory();
    }
    
    /*
     * This method add the right job in the jobQueue base on the oldstate and newState of Guidance
     * The actual changing of GuidanceState copy is happened after each job is finished
     */
    @Override
    public void onStateChange(GuidanceAction action) {
        switch (action) {
        case INTIALIZE:
            jobQueue.add(this::onSystemReady);
            break;
        case ACTIVATE:
            jobQueue.add(this::onRouteActive);
            break;
        case DEACTIVATE:
            jobQueue.add(this::onDeactivate);
            break;
        case ENGAGE:
            jobQueue.add(this::onEngaged);
            break;
        case SHUTDOWN:
            jobQueue.add(this::onShutdown);
            break;
        case PANIC_SHUTDOWN:
            jobQueue.add(this::onPanic);
            break;
        case RESTART:
            jobQueue.add(this::onCleanRestart);
            break;
        default:
            throw new RosRuntimeException(getComponentName() + "received unknow instruction from guidance state machine.");
        }
    }
}
