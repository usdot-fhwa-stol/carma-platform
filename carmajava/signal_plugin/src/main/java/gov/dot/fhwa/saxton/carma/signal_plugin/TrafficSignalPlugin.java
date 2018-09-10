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

package gov.dot.fhwa.saxton.carma.signal_plugin;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicLong;
import java.util.concurrent.atomic.AtomicReference;

import org.joda.time.DateTime;
import org.ros.message.MessageFactory;
import org.ros.message.Time;
import org.ros.node.NodeConfiguration;

import cav_msgs.Connection;
import cav_msgs.GenericLane;
import cav_msgs.IntersectionState;
import cav_msgs.MovementState;
import cav_msgs.NodeListXY;
import cav_msgs.NodeOffsetPointXY;
import cav_msgs.NodeXY;
import cav_msgs.Position3D;
import cav_msgs.TrafficSignalInfo;
import cav_msgs.TrafficSignalInfoList;
import cav_msgs.UIInstructions;
import geometry_msgs.TwistStamped;
import gov.dot.fhwa.saxton.carma.guidance.arbitrator.TrajectoryPlanningResponse;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.IManeuver;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.LongitudinalManeuver;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.ManeuverType;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.SlowDown;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.SpeedUp;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.SteadySpeed;
import gov.dot.fhwa.saxton.carma.guidance.plugins.AbstractPlugin;
import gov.dot.fhwa.saxton.carma.guidance.plugins.IStrategicPlugin;
import gov.dot.fhwa.saxton.carma.guidance.plugins.PluginServiceLocator;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.IPublisher;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.IService;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.IServiceServer;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.ISubscriber;
import gov.dot.fhwa.saxton.carma.guidance.trajectory.Trajectory;
import gov.dot.fhwa.saxton.carma.guidance.util.IntersectionData;
import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.Constants;
import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.DataElementHolder;
import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.DataElementKey;
import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.DoubleDataElement;
import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.GlidepathAppConfig;
import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.IntersectionCollectionDataElement;
import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.SignalPhase;
import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.utils.GlidepathApplicationContext;
import gov.dot.fhwa.saxton.carma.signal_plugin.asd.IntersectionCollection;
import gov.dot.fhwa.saxton.carma.signal_plugin.asd.Lane;
import gov.dot.fhwa.saxton.carma.signal_plugin.asd.Location;
import gov.dot.fhwa.saxton.carma.signal_plugin.asd.map.MapMessage;
import gov.dot.fhwa.saxton.carma.signal_plugin.asd.spat.LaneSet;
import gov.dot.fhwa.saxton.carma.signal_plugin.asd.spat.Movement;
import gov.dot.fhwa.saxton.carma.signal_plugin.asd.spat.SpatMessage;
import gov.dot.fhwa.saxton.carma.signal_plugin.ead.EadAStar;
import gov.dot.fhwa.saxton.carma.signal_plugin.ead.trajectorytree.Node;
import gov.dot.fhwa.saxton.carma.signal_plugin.filter.PolyHoloA;
import sensor_msgs.NavSatFix;
import std_msgs.Bool;
import std_srvs.SetBool;
import std_srvs.SetBoolRequest;
import std_srvs.SetBoolResponse;

/**
 * Top level class in the Traffic Signal Plugin that does trajectory planning through
 * signalized intersections. This is a port of the functionality developed under
 * the STOL I contract TO 17 and STOL II contract TO 13, Glidepath project.
 */
public class TrafficSignalPlugin extends AbstractPlugin implements IStrategicPlugin {

    private ISubscriber<NavSatFix> gpsSub;
    private ISubscriber<TwistStamped> velocitySub;
    private IPublisher<TrafficSignalInfoList> trafficSignalInfoPub;
    private IPublisher<UIInstructions> uiInstructionsPub;
    private AtomicBoolean awaitingUserInput = new AtomicBoolean(false);
    private AtomicBoolean awaitingUserConfirmation = new AtomicBoolean(false);
    private AtomicLong prevUIRequestTime = new AtomicLong();
    private final long UI_REQUEST_INTERVAL = 100;
    private final String GO_BUTTON_SRVS = "traffic_signal_plugin/go_button";
    private static final double ZERO_SPEED_NOISE = 0.04;	//speed threshold below which we consider the vehicle stopped, m/s
    private MessageFactory messageFactory = NodeConfiguration.newPrivate().getTopicMessageFactory();
    private final int NUM_SIGNALS_ON_UI = 3;

    private Map<Integer, IntersectionData> intersections = Collections
            .synchronizedMap(new HashMap<Integer, IntersectionData>());
    private AtomicReference<NavSatFix> curPos = new AtomicReference<>();
    private AtomicReference<TwistStamped> curVel = new AtomicReference<>();
    private gov.dot.fhwa.saxton.carma.signal_plugin.ead.Trajectory glidepathTrajectory;
    private PolyHoloA velFilter = new PolyHoloA();
    private EadAStar ead;
    private double operSpeedScalingFactor = 1.0;
    private double speedCommandQuantizationFactor = 0.1;
    private AtomicBoolean involvedInControl = new AtomicBoolean(false);

    public TrafficSignalPlugin(PluginServiceLocator psl) {
        super(psl);
        version.setName("Traffic Signal Plugin");
        version.setMajorRevision(1);
        version.setIntermediateRevision(0);
        version.setMinorRevision(0);
    }

    @Override
    public void onInitialize() {
        // load params
        // Pass params into GlidepathAppConfig
        GlidepathAppConfig appConfig = new GlidepathAppConfig(pluginServiceLocator.getParameterSource(), pluginServiceLocator.getRouteService());
        GlidepathApplicationContext.getInstance().setAppConfigOverride(appConfig);

        // Initialize Speed Filter
        velFilter.initialize(appConfig.getPeriodicDelay() * Constants.MS_TO_SEC); // TODO determine what the best value should be given we no longer use the periodic executor
        
        // log the key params here
        pluginServiceLocator.getV2IService().registerV2IDataCallback(this::handleNewIntersectionData);
        setAvailability(false);
        gpsSub = pluginServiceLocator.getPubSubService().getSubscriberForTopic("nav_sat_fix", NavSatFix._TYPE);
        gpsSub.registerOnMessageCallback((msg) -> {
            curPos.set(msg);
            if (checkIntersectionMaps()) {
                if (involvedInControl.compareAndSet(false, true)) {
                    triggerNewPlan(true);
                    log.info("On map and replanning");
                }
            } else if (involvedInControl.compareAndSet(true, false)) {
                log.info("Off map requesting replan");
                triggerNewPlan(false);
            }
        });

        velocitySub = pluginServiceLocator.getPubSubService().getSubscriberForTopic("velocity", TwistStamped._TYPE);
        velocitySub.registerOnMessageCallback((msg) -> {
            curVel.set(msg);
            velFilter.addRawDataPoint(msg.getTwist().getLinear().getX());
        });

        uiInstructionsPub = pluginServiceLocator.getPubSubService().getPublisherForTopic("ui_instructions", UIInstructions._TYPE);

        trafficSignalInfoPub = pluginServiceLocator.getPubSubService().getPublisherForTopic("traffic_signal_info", TrafficSignalInfoList._TYPE);

        pubSubService.createServiceServerForTopic(GO_BUTTON_SRVS, SetBool._TYPE, 
            (SetBoolRequest request, SetBoolResponse response) -> {
                if (!awaitingUserInput.compareAndSet(true, false)) {
                    response.setMessage(this.getVersionInfo().componentName() + " did not expect UI input and is ignoring the input.");
                    response.setSuccess(false);
                    return; // If we are not expecting user acknowledgement then don't continue
                }
                if (request.getData()) { // If user acknowledged it is safe to continue
                    if (checkCurrentPhase(SignalPhase.GREEN)) { // If we are still in the green phase
                        // Notify the arbitrator we need to replan to continue through the intersection
                        triggerNewPlan(true);
                    } else {
                        response.setMessage("The light is no longer green. Waiting till next green light.");
                        response.setSuccess(false);
                        return;
                    }
                } else { // User indicated it is not safe to continue
                    prevUIRequestTime.set(System.currentTimeMillis()); // TODO should this be ROS time
                }

                response.setSuccess(true); // If we reach this point the service request has been handled
            }
        );

        log.info("STARTUP", "TrafficSignalPlugin has been initialized.");
    }

    /**
     * Add node offset to Lane object
     * 
     * @param lane Lane to add the offset to
     * @param x X value of the offset in meters
     * @param y Y value of the offset in meters
     */
    private void addNodeOffset(Lane lane, Location ref, float x, float y) {
        lane.addNodeCm(ref, (int) (x * 100), (int) (y * 100));
    }

    /**
     * Convert IntersectionData instance into old Glidepath app object
     * @param data The {@class IntersectionData} instance to be converted
     * @return The map data from input converted into a Glidepath formatted object
     */
    private MapMessage convertMapMessage(IntersectionData data) {
        MapMessage map = new MapMessage();
        map.setContentVersion(data.getIntersectionGeometry().getRevision());
        map.setElevationsPresent(false);
        map.setOffsetsInDm(false);

        map.setIntersectionId(data.getIntersectionId());

        Position3D ref = data.getIntersectionGeometry().getRefPoint();
        Location refPoint = new Location(ref.getLatitude(), ref.getLongitude());
        map.setRefPoint(refPoint);

        // Convert lanes
        List<Lane> laneList = new ArrayList<>();
        for (GenericLane laneData : data.getIntersectionGeometry().getLaneSet().getLaneList()) {
            Lane cnvLane = new Lane();
            cnvLane.setId(laneData.getLaneId());

            cnvLane.setApproach(laneData.getIngressApproachExists());
            cnvLane.setAttributes(0);

            if (data.getIntersectionGeometry().getLaneWidthExists()) {
                cnvLane.setWidth(data.getIntersectionGeometry().getLaneWidth());
            }

            if (laneData.getNodeList().getChoice() == NodeListXY.NODE_SET_XY) {
                for (NodeXY nodeBase : laneData.getNodeList().getNodes().getNodeSetXy()) {
                    NodeOffsetPointXY delta = nodeBase.getDelta();
                    switch (delta.getChoice()) {
                    case NodeOffsetPointXY.NODE_XY1:
                        addNodeOffset(cnvLane, refPoint, delta.getNodeXy1().getX(), delta.getNodeXy1().getY());
                        break;
                    case NodeOffsetPointXY.NODE_XY2:
                        addNodeOffset(cnvLane, refPoint, delta.getNodeXy2().getX(), delta.getNodeXy2().getY());
                        break;
                    case NodeOffsetPointXY.NODE_XY3:
                        addNodeOffset(cnvLane, refPoint, delta.getNodeXy3().getX(), delta.getNodeXy3().getY());
                        break;
                    case NodeOffsetPointXY.NODE_XY4:
                        addNodeOffset(cnvLane, refPoint, delta.getNodeXy4().getX(), delta.getNodeXy4().getY());
                        break;
                    case NodeOffsetPointXY.NODE_XY5:
                        addNodeOffset(cnvLane, refPoint, delta.getNodeXy5().getX(), delta.getNodeXy5().getY());
                        break;
                    case NodeOffsetPointXY.NODE_XY6:
                        addNodeOffset(cnvLane, refPoint, delta.getNodeXy6().getX(), delta.getNodeXy6().getY());
                        break;
                    case NodeOffsetPointXY.NODE_LATLON:
                        Location node = new Location(delta.getNodeLatlon().getLatitude(),
                                delta.getNodeLatlon().getLongitude());
                        cnvLane.addNodeLatLon(node);
                        break;
                    default:
                        break;
                    }
                }
            }
        }
        map.setLanes(laneList);

        return map;
    }

    /**
     * Convert IntersectionData instance into old Glidepath app object
     * @param data The {@class IntersectionData} instance to be converted
     * @return The SPAT data from input converted into a Glidepath formatted object
     */
    private SpatMessage convertSpatMessage(IntersectionData data) {
        IntersectionState state = data.getIntersectionState();
        SpatMessage cnvSpat = new SpatMessage();

        cnvSpat.setContentVersion(state.getRevision());
        cnvSpat.setIntersectionId(state.getId().getId());

        cnvSpat.setStatus(state.getStatus().getIntersectionStatusObject());
        cnvSpat.setTimeStamp(DateTime.now()); // TODO: Improve estimation of data age

        List<Movement> movements = new ArrayList<>();
        for (MovementState movementData : state.getStates().getMovementList()) {
            Movement m = new Movement();

            for (GenericLane lane : data.getIntersectionGeometry().getLaneSet().getLaneList()) {
                // Detect based on connectsTo
                if (lane.getConnectsToExists()) {
                    for (Connection connectsTo : lane.getConnectsTo().getConnectToList()) {
                        if (connectsTo.getSignalGroupExists()) {
                            if (connectsTo.getSignalGroup() == movementData.getSignalGroup()
                                    && connectsTo.getConnectingLane().getManeuverExists()
                                    && connectsTo.getConnectingLane().getManeuver().getAllowedManeuvers() == 0) {
                                LaneSet lanes = new LaneSet(lane.getLaneId(), 0x01); // TODO: Detect maneuvers other
                                                                                     // than
                                                                                     // straight
                                m.addLaneSet(lanes);
                            }
                        }
                    }
                }
            }

            movements.add(m);
        }
        cnvSpat.setMovements(movements);

        return cnvSpat;
    }

    /**
     * Helper function to trigger a new plan from arbitrator to accommodate for Glidepath planning
     * 
     * @param availability The availability to set when calling this replan
     */
    private void triggerNewPlan(boolean availability) {
        log.info("Requesting replan with availability: " + availability);
        setAvailability(availability);
        pluginServiceLocator.getArbitratorService().requestNewPlan();
    }

    /**
     * Callback receiver for new intersection data from {@link V2IService}
     */
    private void handleNewIntersectionData(List<IntersectionData> data) {
        synchronized (intersections) {
            boolean phaseChanged = false;
            boolean newIntersection = false;
            List<Integer> foundIds = new LinkedList<>();
            for (IntersectionData datum : data) {
                foundIds.add(datum.getIntersectionId()); // Track the visible intersection ids
                if (!intersections.containsKey(datum.getIntersectionId())) {
                    intersections.put(datum.getIntersectionId(), datum);
                    newIntersection = true;
                } else {
                    // If it is in the list, check to see if this has changed the phase
                    IntersectionData old = intersections.get(datum.getIntersectionId());
                    intersections.put(datum.getIntersectionId(), datum);

                    /**
                     * This code is looking like it's O(bad) asymptotic complexity in the worst
                     * case, but I'm not sure it's an issue as we're unlikely to experience very
                     * large numbers of movements in a single intersection. But in the event
                     * performance is an issue this might be a likely culprit to investigate.
                     * 
                     * -KR
                     */

                    // Walk through all the movements to compare
                    IntersectionState oldIntersectionState = old.getIntersectionState();
                    if (oldIntersectionState == null) {
                        log.warn("Intersection could not be processed because it has no state: " + old);
                        continue;
                    }
                    phaseChangeCheckLoop: for (MovementState oldMov : oldIntersectionState.getStates()
                            .getMovementList()) {
                        for (MovementState newMov : datum.getIntersectionState().getStates().getMovementList()) {
                            if (oldMov.getSignalGroup() == newMov.getSignalGroup()) {
                                // If we find a shared movement, check the movement events for sameness

                                // ASSUMPTION: Phase states within a movement will always be reported in the same order
                                // There is no ID for an individual movement event within a movement, so I can only 
                                // compare different messages based on positional similarity.
                                for (int i = 0; i < oldMov.getStateTimeSpeed().getMovementEventList().size(); i++) {
                                    if (i >= newMov.getStateTimeSpeed().getMovementEventList().size()
                                            || (oldMov.getStateTimeSpeed().getMovementEventList().get(i)
                                                    .getEventState() != newMov.getStateTimeSpeed()
                                                            .getMovementEventList().get(i).getEventState())) {
                                        // Phase change detected, a replan will be needed
                                        phaseChanged = true;
                                        break phaseChangeCheckLoop; // Break outer for loop labeled phaseChangeCheckLoop
                                    }

                                }
                            }
                        }
                    }
                }
            }

            // Remove expired intersections
            int prevNumIntersections = intersections.entrySet().size();
            intersections.entrySet().removeIf(entry -> !foundIds.contains(entry.getKey()));
            boolean deletedIntersection = intersections.entrySet().size() != prevNumIntersections;

            // Trigger new plan on phase change
            if ((phaseChanged || newIntersection || deletedIntersection) && checkIntersectionMaps()) {
                triggerNewPlan(true);
            }
        }
    }

    /**
     * Compute the DTSB value if we're on a known MAP message
     * 
     * @return -1 if DTSB check fails for any reason, Double.MAX_VALUE if we're not on a MAP message
     */
    private double computeDtsb() {
        NavSatFix posMsg = curPos.get();
        if (posMsg == null) {
            log.warn("NavSatFix was null");
            return -1;
        }
        if (glidepathTrajectory == null) {
            log.warn("Traj was null");
            return -1;
        }
        try {
            Location curLoc = new Location(curPos.get().getLatitude(), curPos.get().getLongitude());
            double dtsb = glidepathTrajectory.updateIntersections(convertIntersections(intersections), curLoc);
            updateUISignals();
            return dtsb;
        } catch (Exception e) {
            log.warn("DTSB computation failed!", e);
        }

        return -1;
    }

    /**
     * Helper function to update the Traffic Signal Plugin UI with new intersection data.
     * This function is meant to be called after a call to Trajectory.updateIntersections
     */
    private void updateUISignals() {
        TrafficSignalInfoList msg = trafficSignalInfoPub.newMessage();
        int displayCount = Math.min(NUM_SIGNALS_ON_UI, glidepathTrajectory.getSortedIntersections().size());
        for(int i = 0; i < displayCount; i++) {
            final TrafficSignalInfo signalMsg = intersectionDataToMsg(glidepathTrajectory.getSortedIntersections().get(i));
            msg.getTrafficSignalInfoList().add(signalMsg);
        }
        trafficSignalInfoPub.publish(msg);
    }

    /**
     * Helper function builds a TrafficSignalInfo message from IntersectionData
     * 
     * @param intersection The intersection to convert to a message
     * @return The fully defined message
     */
    private TrafficSignalInfo intersectionDataToMsg(gov.dot.fhwa.saxton.carma.signal_plugin.asd.IntersectionData intersection) {
        TrafficSignalInfo signalMsg = messageFactory.newFromType(TrafficSignalInfo._TYPE);
        signalMsg.setIntersectionId((short)intersection.intersectionId);
        signalMsg.setLaneId((short)intersection.laneId);
        signalMsg.setRemainingDistance((float)intersection.dtsb);
        signalMsg.setRemainingTime((short)intersection.timeToNextPhase);
        switch(intersection.currentPhase) {
            case GREEN:
                signalMsg.setState(TrafficSignalInfo.GREEN);
                break;
            case YELLOW:
                signalMsg.setState(TrafficSignalInfo.YELLOW);
                break;
            case RED:
                signalMsg.setState(TrafficSignalInfo.RED);
                break;
            default: // Leave light off
                break;
        }
        return signalMsg;
    }

    /**
     * Check to see if we're on a MAP message
     * 
     * @return true, if we're on a MAP message, false O.W.
     */
    private boolean checkIntersectionMaps() {
        double dtsb = computeDtsb();
        log.info("Computed DTSB: " + dtsb);
        return dtsb > 0 && dtsb < Double.MAX_VALUE;
    }

    @Override
    public void onResume() {
        log.info("TrafficSignalPlugin trying to resume.");
        ead = new EadAStar();
        try {
            glidepathTrajectory = new gov.dot.fhwa.saxton.carma.signal_plugin.ead.Trajectory(ead);
        } catch (Exception e) {
            log.error("Unable to initialize EAD algorithm!!!", e);
            setAvailability(false);
            return;
        }
        pluginServiceLocator.getV2IService().registerV2IDataCallback(this::handleNewIntersectionData);
        log.info("TrafficSignalPlugin has resumed.");
    }

    @Override
    public void loop() throws InterruptedException {
        long tsStart = System.currentTimeMillis();
        
        // We were stopped at a red light and now the light is green
        boolean stoppedForLight = stoppedAtLight();
        boolean phaseIsGreen = checkCurrentPhase(SignalPhase.GREEN);
        if (stoppedForLight && phaseIsGreen) {
            // If we have not requested acknowledgement from the user do it now
            if (awaitingUserConfirmation.compareAndSet(false, true)) {
                // Update state variables
                prevUIRequestTime.set(System.currentTimeMillis());
                awaitingUserInput.set(true);
                // Send user input request to get confirmation to continue
                askUserIfIntersectionIsClear();

            } else if (System.currentTimeMillis() - UI_REQUEST_INTERVAL > prevUIRequestTime.get() // Already awaiting confirmation and enough time has elapsed
                && awaitingUserInput.compareAndSet(false, true)) { // AND we are not currently waiting for user input
                // Then we want to request input from the user again
                askUserIfIntersectionIsClear();
            }
        } else if (!stoppedForLight && awaitingUserConfirmation.compareAndSet(true, false)) {
            // We are no longer waiting at the intersection
        }

        long tsEnd = System.currentTimeMillis();
        long sleepDuration = Math.max(100 - (tsEnd - tsStart), 0);
        Thread.sleep(sleepDuration);
    }

    /**
     * Helper function which sends a acknowledgement request to the UI which will request input from the user
     */
    private void askUserIfIntersectionIsClear() {
        UIInstructions uiMsg = uiInstructionsPub.newMessage();
        uiMsg.setType(UIInstructions.ACK_REQUIRED);
        uiMsg.setMsg("Is it safe to continue through intersection?");
        uiMsg.setResponseService(GO_BUTTON_SRVS);
        uiMsg.setStamp(Time.fromMillis(System.currentTimeMillis()));
        uiInstructionsPub.publish(uiMsg);
    }

    /**
     * Helper function returns true if the vehicle is currently stopped at a light based on an plan from this plugin
     * 
     * @return True if the current maneuver is a steady speed stop maneuver planned by this plugin
     */
    private boolean stoppedAtLight() {
        double currentSpeed = pluginServiceLocator.getManeuverPlanner().getManeuverInputs().getCurrentSpeed();
        LongitudinalManeuver currentLongitudinalManeuver = (LongitudinalManeuver) pluginServiceLocator.getArbitratorService().getCurrentlyExecutingManeuver(ManeuverType.LONGITUDINAL);
        if (currentLongitudinalManeuver == null) {
            return false; // Can't be stopped at a light if we are not under automated control or in a complex maneuver which we would not have planned
        }
        currentLongitudinalManeuver.getPlanner().equals(this);
        
        return currentLongitudinalManeuver.getPlanner().equals(this) 
            && currentLongitudinalManeuver instanceof SteadySpeed
            && Math.abs(currentLongitudinalManeuver.getTargetSpeed()) < ZERO_SPEED_NOISE
            && Math.abs(currentSpeed) < ZERO_SPEED_NOISE;
    }

    /**
     * Helper function returns true if the current phase of the nearest intersection is equal to the requested phase
     * 
     * @param phase The phase to check against
     * 
     * @return True if nearest intersection phase is equal to current phase
     */
    private boolean checkCurrentPhase(SignalPhase phase) {
        List<gov.dot.fhwa.saxton.carma.signal_plugin.asd.IntersectionData> sortedIntersections = glidepathTrajectory.getSortedIntersections();
        return !sortedIntersections.isEmpty() && sortedIntersections.get(0).currentPhase == phase;
    }

    @Override
    public void onSuspend() {

        log.info("SignalPlugin has been suspended.");
    }

    @Override
    public void onTerminate() {
    }

    private List<gov.dot.fhwa.saxton.carma.signal_plugin.asd.IntersectionData> convertIntersections(
            Map<Integer, IntersectionData> data) {
        List<gov.dot.fhwa.saxton.carma.signal_plugin.asd.IntersectionData> out = new ArrayList<>();
        for (IntersectionData datum : data.values()) {
            gov.dot.fhwa.saxton.carma.signal_plugin.asd.IntersectionData converted = new gov.dot.fhwa.saxton.carma.signal_plugin.asd.IntersectionData();
            converted.map = convertMapMessage(datum);
            converted.spat = convertSpatMessage(datum);

            out.add(converted);
        }

        return out;
    }

    @Override
    public TrajectoryPlanningResponse planTrajectory(Trajectory traj, double expectedStartSpeed) {
        log.info("Entered planTrajectory");
        // CHECK TRAJECTORY LENGTH
        double dtsb = computeDtsb();

        if (dtsb > 0 && dtsb < Double.MAX_VALUE) {
            // DTSB computation successful, check to see if we can plan up to stop bar
            if (traj.getStartLocation() + (dtsb * 1.1) > traj.getEndLocation()) {
                // Not enough distance to allow for proper glidepath execution
                TrajectoryPlanningResponse tpr = new TrajectoryPlanningResponse();
                tpr.requestLongerTrajectory(traj.getStartLocation() + (dtsb * 1.1)); // allow for some extra slack
                return tpr;
            }
        }

        log.info("DTSB within traj");

        // CHECK PLANNING PRIORITY
        if (!traj.getLongitudinalManeuvers().isEmpty()) {
                TrajectoryPlanningResponse tpr = new TrajectoryPlanningResponse();
                tpr.requestHigherPriority();
                return tpr;
        }

        log.info("Granted highest planning priority");

        // CONVERT DATA ELEMENTS
        DataElementHolder state = new DataElementHolder();
        DoubleDataElement curSpeedElement;
        DoubleDataElement curAccelElement;
        DoubleDataElement operSpeedElem;
        DoubleDataElement vehicleLat;
        DoubleDataElement vehicleLon;
        IntersectionCollectionDataElement icde;

        if (curVel.get() != null) {
            curSpeedElement = new DoubleDataElement(velFilter.getSmoothedValue());
            curAccelElement = new DoubleDataElement(velFilter.getSmoothedDerivative());
        } else {
            curSpeedElement = new DoubleDataElement(0.0);
            curAccelElement = new DoubleDataElement(0.0);
        }
        state.put(DataElementKey.SMOOTHED_SPEED, curSpeedElement);
        state.put(DataElementKey.ACCELERATION, curAccelElement);
        operSpeedElem = new DoubleDataElement(
                pluginServiceLocator.getRouteService().getSpeedLimitAtLocation(traj.getStartLocation()).getLimit()
                        * operSpeedScalingFactor);
        state.put(DataElementKey.OPERATING_SPEED, operSpeedElem);

        vehicleLat = new DoubleDataElement(curPos.get().getLatitude());
        vehicleLon = new DoubleDataElement(curPos.get().getLongitude());
        state.put(DataElementKey.LATITUDE, vehicleLat);
        state.put(DataElementKey.LONGITUDE, vehicleLon);

        IntersectionCollection ic = new IntersectionCollection();
        ic.intersections = convertIntersections(intersections);
        icde = new IntersectionCollectionDataElement(ic);
        state.put(DataElementKey.INTERSECTION_COLLECTION, icde);

        log.info("Requesting AStar plan with intersections: " + intersections.toString());

        try {
            glidepathTrajectory.getSpeedCommand(state); // TODO do we really need the trajectory object. It's purpose is 99% filled by the plugin
        } catch (Exception e) {
            log.error("Glidepath trajectory planning threw exception!", e);
            TrajectoryPlanningResponse tpr = new TrajectoryPlanningResponse();
            tpr.requestHigherPriority(); // indicate generic failure
            return tpr;
        }

        // GET NODES OUT OF EADASTAR
        List<Node> eadResult = ead.getCurrentPath();

        if (eadResult == null) {
            log.warn("Ead result is null");
            return new TrajectoryPlanningResponse();
        } else {
            log.info("Ead result is path of size: " + eadResult.size());
        }

        /*
         * Optimization has been decided against due to complexities in trajectory behavior.
         * Will be re-explored if performance issues arise as a result of the way a trajectory
         * is constructed out of many small maneuvers
         * 
         * - KR
         */

        // OPTIMIZE NODES
        // List<Node> optimizedOutput = new ArrayList<>();

        /*if (eadResult.size() > 2) {
            Node prev = eadResult.get(0);
            optimizedOutput.add(prev);
            for (int i = 1; i < eadResult.size(); i++) {
                Node cur = eadResult.get(i);
                if (Math.abs(prev.getSpeedAsDouble() - cur.getSpeedAsDouble()) >= speedCommandQuantizationFactor
                        || i == eadResult.size() - 1) {
                    // If speed is above quantization threshold or point is last in series
                    optimizedOutput.add(cur);
                    prev = cur;
                }
            }
        } */

        // CONVERT AND INSERT MANEUVERS
        double startDist = traj.getStartLocation();
        Node prev = null;
        for (Node cur : eadResult) {
            if (prev == null) {
                prev = cur;
            } else {
                if (Math
                        .abs(prev.getSpeedAsDouble() - cur.getSpeedAsDouble()) < speedCommandQuantizationFactor) {
                    SteadySpeed steadySpeed = new SteadySpeed(this);

                    if (cur.getSpeedAsDouble() > speedCommandQuantizationFactor) {
                        steadySpeed.setSpeeds(cur.getSpeedAsDouble(), cur.getSpeedAsDouble());
                        pluginServiceLocator.getManeuverPlanner().planManeuver(steadySpeed,
                                startDist + prev.getDistanceAsDouble(), startDist + cur.getDistanceAsDouble());
                    } else {
                        // We're coming to a stop, so plan an indefinite length stop maneuver, to be
                        // overridden by replan later
                        steadySpeed.setSpeeds(0.0, 0.0);
                        pluginServiceLocator.getManeuverPlanner().planManeuver(steadySpeed,
                                startDist + prev.getDistanceAsDouble(), traj.getEndLocation());
                        break;
                    }

                    traj.addManeuver(steadySpeed);
                } else if (prev.getSpeedAsDouble() < cur.getSpeedAsDouble()) {
                    SpeedUp speedUp = new SpeedUp(this);
                    speedUp.setSpeeds(prev.getSpeedAsDouble(), cur.getSpeedAsDouble());
                    pluginServiceLocator.getManeuverPlanner().planManeuver(speedUp,
                            startDist + prev.getDistanceAsDouble(), startDist + cur.getDistanceAsDouble());
                    traj.addManeuver(speedUp);
                } else {
                    SlowDown slowDown = new SlowDown(this);
                    slowDown.setSpeeds(prev.getSpeedAsDouble(), cur.getSpeedAsDouble());
                    pluginServiceLocator.getManeuverPlanner().planManeuver(slowDown,
                            startDist + prev.getDistanceAsDouble(), startDist + cur.getDistanceAsDouble());
                    traj.addManeuver(slowDown);
                } 
            }
        }

        log.info("Planning complete");
        setAvailability(false);
        return new TrajectoryPlanningResponse();
    }
}
