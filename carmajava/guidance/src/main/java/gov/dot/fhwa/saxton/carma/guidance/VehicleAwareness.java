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

package gov.dot.fhwa.saxton.carma.guidance;

import java.util.ArrayList;
import java.util.List;
import java.util.UUID;

import org.ros.exception.RosRuntimeException;
import org.ros.message.MessageFactory;
import org.ros.node.ConnectedNode;

import cav_msgs.MobilityPath;
import gov.dot.fhwa.saxton.carma.guidance.conflictdetector.ConflictSpace;
import gov.dot.fhwa.saxton.carma.guidance.conflictdetector.IConflictDetector;
import gov.dot.fhwa.saxton.carma.guidance.mobilityrouter.MobilityPathHandler;
import gov.dot.fhwa.saxton.carma.guidance.plugins.IPlugin;
import gov.dot.fhwa.saxton.carma.guidance.plugins.PluginManager;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.IPubSubService;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.IPublisher;
import gov.dot.fhwa.saxton.carma.guidance.trajectory.Trajectory;
import gov.dot.fhwa.saxton.carma.guidance.trajectory.TrajectoryExecutor;
import gov.dot.fhwa.saxton.carma.guidance.util.ExecutionTimer;
import gov.dot.fhwa.saxton.carma.guidance.util.trajectoryconverter.RoutePointStamped;
import gov.dot.fhwa.saxton.carma.guidance.util.trajectoryconverter.TrajectoryConverter;

/**
 * Class responsible for being aware of nearby vehicles and forcing a replan of our current trajectory
 * if we detect and planned conflict. Also responsible for periodic broadcasts of our path prediction for
 * other vehicles to be aware of our plan via the MobilityPath message.
 */
public class VehicleAwareness extends GuidanceComponent implements IStateChangeListener {
    protected long pathPublishInterval = 3000;
    protected int maxPointsPerMessage = 60;
    protected Trajectory currentTrajectory = null;
    protected Trajectory nextTrajectory = null;
    protected TrajectoryConverter trajectoryConverter;
    protected TrajectoryExecutor trajectoryExecutor;
    protected IConflictDetector conflictDetector;
    protected String conflictHandlerName = "Yield Plugin";
    protected MobilityPathHandler conflictHandler;
    protected String mobilitySenderId = "UNKNOWN";
    protected String currentBsmId = "";
    protected static final String BROADCAST_RECIPIENT_ID = "";
    protected IPublisher<MobilityPath> pathPub;
    protected PluginManager pluginManager;

    public VehicleAwareness(GuidanceStateMachine stateMachine, IPubSubService pubSubService, ConnectedNode node,
            TrajectoryConverter converter, IConflictDetector conflictDetector) {
        super(stateMachine, pubSubService, node);
        stateMachine.registerStateChangeListener(this);
        this.trajectoryConverter = converter;
        this.conflictDetector = conflictDetector;
    }

    /*
     * @param pluginManager the pluginManager to set
     */
    public void setPluginManager(PluginManager pluginManager) {
        this.pluginManager = pluginManager;
    }

    /**
     * @param trajectoryExecutor the trajectoryExecutor to set
     */
    public void setTrajectoryExecutor(TrajectoryExecutor trajectoryExecutor) {
        this.trajectoryExecutor = trajectoryExecutor;
    }

    @Override
    public String getComponentName() {
        return "VehicleAwareness";
    }

    @Override
    public void onStartup() {
        log.info("Loading config params and initing publisher");
        pathPublishInterval = node.getParameterTree().getInteger("~mobility_path_publish_interval", 3000);
        maxPointsPerMessage = node.getParameterTree().getInteger("~mobility_path_max_points", 60);
        mobilitySenderId = node.getParameterTree().getString("~vehicle_id", "UNKNOWN");
        conflictHandlerName = node.getParameterTree().getString("~default_mobility_conflict_handler", "Yield Plugin");
        log.info(String.format(
                "VehicleAwareness init'd with pathPublishInterval=%d, maxPointsPerMessage=%d, mobilitySenderId=%s, conflictHandlerName=%s",
                pathPublishInterval, maxPointsPerMessage, mobilitySenderId, conflictHandlerName));
        pathPub = pubSubService.getPublisherForTopic("outgoing_mobility_path", MobilityPath._TYPE);
    }

    @Override
    public void onSystemReady() {
        log.info("Looking for default conflict handler.");
        for (IPlugin p : pluginManager.getRegisteredPlugins()) {
            if (p.getVersionInfo().componentName().equals(conflictHandlerName)) {
                conflictHandler = (MobilityPathHandler) p;
            }
        }

        if (conflictHandler == null) {
            log.warn("No default conflict handler detected by name: " + conflictHandlerName
                    + ". Guidance will fail on first detected conflict!!!");
        }

        trajectoryExecutor.registerOnTrajectoryProgressCallback(0.0, (pct) -> rollTrajectoryBuffer());
    }

    protected synchronized void rollTrajectoryBuffer() {
        if (nextTrajectory != null) { // Don't roll if this is the first trajectory
            currentTrajectory = nextTrajectory;
            nextTrajectory = null;
            publishMobilityPath();
        }
    }

    @Override
    public void onActive() {

    }

    @Override
    public void onEngaged() {

    }

    @Override
    public void onCleanRestart() {
        log.info("Restarting, cleaning up trajectories");
        currentTrajectory = null;
        nextTrajectory = null;
    }

    @Override
    public void onDeactivate() {

    }

    /**
     * Convert the back and front buffer trajectories into a single set of Route-frame points for
     * conversion and publication in the MobilityPath message.
     */
    private synchronized List<RoutePointStamped> getPathPrediction() {
        List<RoutePointStamped> pathPrediction = new ArrayList<>();
        if (currentTrajectory != null) {
            pathPrediction.addAll(trajectoryConverter.convertToPath(currentTrajectory));
        }
        if (nextTrajectory != null && !pathPrediction.isEmpty()) {
            RoutePointStamped lastPoint = pathPrediction.get(pathPrediction.size() - 1);
            pathPrediction.addAll(trajectoryConverter.convertToPath(nextTrajectory, lastPoint, maxPointsPerMessage - pathPrediction.size()));
        }

        return pathPrediction;
    }

    /**
     * Method to be called when a new trajectory has been planned by the Arbitrator, triggers conflict detection
     * and replan if needed as well as forces an immediate broadcast of a new MobilityPath message
     */
    public synchronized void notifyNewTrajectoryPlanned(Trajectory traj) {
        log.info(String.format("Received notice of new trajectory from [%.02f, %.02f)", traj.getStartLocation(),
                traj.getEndLocation()));
        if (currentTrajectory == null) {
            log.info("Inserted into front buffer!");
            currentTrajectory = traj;
        } else {
            log.info("Inserted into back buffer!");
            nextTrajectory = traj;
        }

        List<RoutePointStamped> pathPrediction = getPathPrediction();
        List<ConflictSpace> conflicts = conflictDetector.getConflicts(pathPrediction);

        if (!conflicts.isEmpty()) {
            if (conflictHandler == null) {
                throw new RosRuntimeException("Unable to locate default conflict handler: " + conflictHandlerName
                        + ". Cannot handle conflict on current trajectory!!!");
            } else {
                log.info("Conflict detected! Handling by delegating to: " + conflictHandlerName);
                // Just pass it null for now
                new Thread(() -> conflictHandler.handleMobilityPathMessageWithConflict(null, true, conflicts.get(0)),
                 "HandleMobilityPathMessageWithConflictCallback").start();
            }
        } else {
            log.info("No conflicts detected in trajectory.");
        }

        publishMobilityPath(pathPrediction);
    }

    /**
     * Method to be called when the future planned state of the vehicle is no longer valid.
     */
    public synchronized void notifyForcedReplan() {
        log.info("Notified of a forced replan, cleaning invalid trajectories!");
        currentTrajectory = null;
        nextTrajectory = null;
    }

    /**
     * Collects the current path prediction of the vehicle and publishes that as a MobilityPath message
     */
    private synchronized void publishMobilityPath() {
        publishMobilityPath(getPathPrediction());
    }

    /**
     * Publishes the MobilityPath message that corresponds to the input path prediction data
     */
    private synchronized void publishMobilityPath(List<RoutePointStamped> pathPrediction) {
        if (stateMachine.getState() != GuidanceState.ENGAGED) {
            // Don't bother until we're engaged
            return;
        }

        log.info("Beginning publication of mobility path...");
        MessageFactory factory = node.getTopicMessageFactory();
        MobilityPath pathMsg = factory.newFromType(MobilityPath._TYPE);
        // Set message header data
        pathMsg.getHeader().setSenderId(mobilitySenderId);
        pathMsg.getHeader().setRecipientId(BROADCAST_RECIPIENT_ID);
        pathMsg.getHeader().setTimestamp(System.currentTimeMillis());
        pathMsg.getHeader().setPlanId(UUID.randomUUID().toString());

        // TODO: Figure out how to get currentBsmId, I don't think we need this yet though
        pathMsg.getHeader().setSenderBsmId(currentBsmId);
        pathMsg.setTrajectory(trajectoryConverter.pathToMessage(pathPrediction));

        pathPub.publish(pathMsg);
        log.info(String.format("Publication complete for planId=%s containing %d points",
                pathMsg.getHeader().getPlanId(), (pathMsg.getTrajectory().getOffsets().size() + 1)));
    }

    @Override
    public void timingLoop() throws InterruptedException {
        ExecutionTimer.runInFixedTime(pathPublishInterval, this::publishMobilityPath);
    }

    @Override
    public void onStateChange(GuidanceAction action) {
        log.debug("GUIDANCE_STATE", getComponentName() + " received action: " + action);
        switch (action) {
        case INTIALIZE:
            jobQueue.add(this::onStartup);
            jobQueue.add(this::onSystemReady);
            break;
        case ACTIVATE:
            jobQueue.add(this::onActive);
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
            throw new RosRuntimeException(
                    getComponentName() + "received unknown instruction from guidance state machine.");
        }
    }
}
