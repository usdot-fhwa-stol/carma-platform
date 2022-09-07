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

package gov.dot.fhwa.saxton.carma.signal_plugin;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Map.Entry;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicLong;
import java.util.concurrent.atomic.AtomicReference;

import com.google.common.util.concurrent.AtomicDouble;

import org.joda.time.DateTime;
import org.ros.message.MessageFactory;
import org.ros.message.Time;
import org.ros.node.NodeConfiguration;
import org.ros.rosjava_geometry.Transform;
import org.ros.rosjava_geometry.Vector3;

import cav_msgs.GenericLane;
import cav_msgs.IntersectionState;
import cav_msgs.MovementEvent;
import cav_msgs.MovementState;
import cav_msgs.NodeListXY;
import cav_msgs.NodeOffsetPointXY;
import cav_msgs.NodeXY;
import cav_msgs.Position3D;
import cav_msgs.TrafficSignalInfo;
import cav_msgs.TrafficSignalInfoList;
import cav_msgs.UIInstructions;
import cav_srvs.GetTransform;
import cav_srvs.GetTransformRequest;
import cav_srvs.GetTransformResponse;
import geometry_msgs.TwistStamped;
import gov.dot.fhwa.saxton.carma.geometry.GeodesicCartesianConverter;
import gov.dot.fhwa.saxton.carma.geometry.cartesian.Point3D;
import gov.dot.fhwa.saxton.carma.guidance.ManeuverPlanner;
import gov.dot.fhwa.saxton.carma.guidance.arbitrator.TrajectoryPlanningResponse;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.IManeuver;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.IManeuverInputs;
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
import gov.dot.fhwa.saxton.carma.guidance.pubsub.OnServiceResponseCallback;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.TopicNotFoundException;
import gov.dot.fhwa.saxton.carma.guidance.trajectory.Trajectory;
import gov.dot.fhwa.saxton.carma.guidance.util.IntersectionData;
import gov.dot.fhwa.saxton.carma.guidance.util.trajectoryconverter.RoutePointStamped;
import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.Constants;
import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.DataElementHolder;
import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.DataElementKey;
import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.DoubleDataElement;
import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.GlidepathAppConfig;
import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.IntersectionCollectionDataElement;
import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.PhaseDataElement;
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
import gov.dot.fhwa.saxton.carma.signal_plugin.ead.PlanInterpolator;
import gov.dot.fhwa.saxton.carma.signal_plugin.ead.trajectorytree.ANAStarSolver;
import gov.dot.fhwa.saxton.carma.signal_plugin.ead.trajectorytree.CoarsePathNeighbors;
import gov.dot.fhwa.saxton.carma.signal_plugin.ead.trajectorytree.Node;
import gov.dot.fhwa.saxton.carma.signal_plugin.filter.PolyHoloA;
import j2735_msgs.MovementPhaseState;
import sensor_msgs.NavSatFix;
import std_msgs.Bool;
import std_srvs.SetBool;
import std_srvs.SetBoolRequest;
import std_srvs.SetBoolResponse;
import cav_msgs.RoadwayEnvironment;

/**
 * Top level class in the Traffic Signal Plugin that does trajectory planning through
 * signalized intersections. This is a port of the functionality developed under
 * the STOL I contract TO 17 and STOL II contract TO 13, Glidepath project.
 */
public class TrafficSignalPlugin extends AbstractPlugin implements IStrategicPlugin, IReplanHandle {

    private ISubscriber<TwistStamped> velocitySub;
    private ISubscriber<RoadwayEnvironment> obstacleSub;
    private IPublisher<TrafficSignalInfoList> trafficSignalInfoPub;
    private IPublisher<UIInstructions> uiInstructionsPub;
    private AtomicBoolean awaitingUserConfirmation = new AtomicBoolean(false);
    private static final String GO_BUTTON_SRVS = "/traffic_signal_plugin/go_button";
    private static final double ZERO_SPEED_NOISE = 0.04;	//speed threshold below which we consider the vehicle stopped, m/s
    private MessageFactory messageFactory = NodeConfiguration.newPrivate().getTopicMessageFactory();
    private static final int NUM_SIGNALS_ON_UI = 3;

    protected IService<GetTransformRequest, GetTransformResponse> getTransformClient;
    private Map<Integer, IntersectionData> intersections = Collections
            .synchronizedMap(new HashMap<Integer, IntersectionData>());
    private AtomicReference<Location> curPos = new AtomicReference<>(); // Only allowed to be null at startup
    private AtomicReference<TwistStamped> curVel = new AtomicReference<>();
    private gov.dot.fhwa.saxton.carma.signal_plugin.ead.Trajectory glidepathTrajectory;
    private PolyHoloA velFilter = new PolyHoloA();
    private EadAStar ead;
    private double operSpeedScalingFactor = 1.0;
    private double speedCommandQuantizationFactor = 0.1;
    private ITrafficSignalPluginCollisionChecker collisionChecker; // Collision checker responsible for tracking NCVs and providing collision checks capabilities
    private AtomicBoolean involvedInControl = new AtomicBoolean(false);
    private double popupOnRedTime;
    private static final double CM_PER_M = 100.0;
    private static final double MAX_DTSB = Integer.MAX_VALUE - 5.0; // Legacy Glidepath code returns Integer.MAX_VALUE for invalid dtsb. Add fudge factor for detecting it as a double

    private static final long LOOP_PERIOD = 100; // Plugin will loop at 10Hz
    private static final GeodesicCartesianConverter gcc = new GeodesicCartesianConverter();
    private TrafficSignalManeuverInputs pluginManeuverInputs; // Custom maneuver inputs used for planning
    private ManeuverPlanner pluginManeuverPlanner; // Maneuver planner used with pluginManeuverInputs for planning
    private double acceptableStopDistance;
    private double twiceAcceptableStopDistance;
    private double defaultSpeedLimit;
    private double defaultAccel;
    private GlidepathAppConfig appConfig;
    private static boolean ignoreLights = false; // Static so it can be used in static spat functions
    private static final int MAX_PLAN_RETRIES = 3; // Maximum number of replanning attempts when a replan request is processed

    // Planning Variables
    private AtomicBoolean replanning = new AtomicBoolean(false);
    private AtomicReference<List<Node>> currentPlan = new AtomicReference<>();
    private AtomicDouble planStartingDowntrack = new AtomicDouble();


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
        appConfig = new GlidepathAppConfig(pluginServiceLocator.getParameterSource(), pluginServiceLocator.getRouteService());
        GlidepathApplicationContext.getInstance().setAppConfigOverride(appConfig);

        ignoreLights = appConfig.getBooleanValue("ead.debug.ignoreLights");

        // Setup the collision checker
        // This must be done before callbacks are created
        if (appConfig.getBooleanValue("ead.handleNCV")) { // Check if we will handle NCVs
            this.collisionChecker = new ObjectCollisionChecker(
                this.pluginServiceLocator,
                new DefaultMotionPredictorFactory(appConfig),
                new PlanInterpolator(),
                this
            );
        } else { // Use No-Op collision checker if not handling NCVs
            this.collisionChecker = new NoOpCollisionChecker();
        }

        popupOnRedTime = appConfig.getDoubleValue("popupOnRedTime");

        // Initialize custom maneuver inputs
        IManeuverInputs platformInputs = pluginServiceLocator.getManeuverPlanner().getManeuverInputs();
        pluginManeuverInputs = new TrafficSignalManeuverInputs(platformInputs, appConfig.getDoubleValue("ead.response.lag"), appConfig.getDoubleValue("defaultAccel") + 1.0);
        pluginManeuverPlanner = new ManeuverPlanner(pluginServiceLocator.getManeuverPlanner().getGuidanceCommands(), pluginManeuverInputs);
        
        acceptableStopDistance = appConfig.getDoubleDefaultValue("ead.acceptableStopDistance", 6.0);

        twiceAcceptableStopDistance = 2.0 * acceptableStopDistance;
        defaultAccel = appConfig.getDoubleValue("defaultAccel");

        // Initialize Speed Filter
        velFilter.initialize(appConfig.getPeriodicDelay() * Constants.MS_TO_SEC); // TODO determine what the best value should be given we no longer use the periodic executor

        // log the key params here
        pluginServiceLocator.getV2IService().registerV2IDataCallback(this::handleNewIntersectionData);
        setAvailability(false);

        velocitySub = pluginServiceLocator.getPubSubService().getSubscriberForTopic("velocity", TwistStamped._TYPE);
        obstacleSub = pluginServiceLocator.getPubSubService().getSubscriberForTopic("roadway_environment", RoadwayEnvironment._TYPE);
        
        uiInstructionsPub = pluginServiceLocator.getPubSubService().getPublisherForTopic("ui_instructions", UIInstructions._TYPE);

        trafficSignalInfoPub = pluginServiceLocator.getPubSubService().getPublisherForTopic("traffic_signal_info", TrafficSignalInfoList._TYPE);

        pubSubService.createServiceServerForTopic(GO_BUTTON_SRVS, SetBool._TYPE, 
            (SetBoolRequest request, SetBoolResponse response) -> {

                //NOTE: awaitingUserConfirmation does not get set in this service response
                //      Instead, awaitingUserConfirmation will be set from true to false during evaluateAtStopping() when stoppedAtLight=false.
                if (request.getData()) { // If user acknowledged it is safe to continue

                    log.info("User confirmed: All Clear");

                    if (checkCurrentPhase(SignalPhase.GREEN)) { // If we are still in the green phase
                        // Notify the arbitrator we need to replan to continue through the intersection
                        log.info("Continuing at new green light");
                        triggerNewPlan(true);
                        response.setSuccess(true);
                        return;
                    } else {
                        log.info("Light not green despite user confirmation");
                        response.setMessage("Sorry, the light is no longer green. When it's safe to continue, please press the YES button.");
                        response.setSuccess(false);
                        return;
                    }

                } else { // User indicated it is not safe to continue
                    log.info("User said it is not safe to continue");
                    response.setMessage("When it's safe to continue, please press the YES button.");
                    response.setSuccess(false);
                    return;
                }

            }
        );

        try {
            getTransformClient = pubSubService.getServiceForTopic("get_transform", GetTransform._TYPE);
        } catch (TopicNotFoundException tnfe) {
            log.error("get_transform service cannot be found", tnfe);
            this.setActivation(false); // TODO is this the best way to handle this error?
        }

        log.info("STARTUP", "TrafficSignalPlugin has been initialized.");
    }

    /**
     * Add node offset to Lane object
     * 
     * @param lane Lane to add the offset to
     * @param x X value of the offset in meters
     * @param y Y value of the offset in meters
     */
    static private void addNodeOffset(Lane lane, Location ref, float x, float y) {
        lane.addNodeCm(ref, (int) (x * 100.0), (int) (y * 100.0));
    }

    /**
     * Convert IntersectionData instance into old Glidepath app object
     * @param data The {@class IntersectionData} instance to be converted
     * @return The map data from input converted into a Glidepath formatted object
     */
    static protected MapMessage convertMapMessage(IntersectionData data) {
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
        for (GenericLane laneData : data.getIntersectionGeometry().getLaneList()) {
            Lane cnvLane = new Lane();
            cnvLane.setId(laneData.getLaneId());

            int dirBitString = laneData.getLaneAttributes().getDirectionalUse().getLaneDirection();

            if (dirBitString == 0) // b00
            {
                // Logging can't be done here since this is a static function
                //log.debug("lane_direction is unset. Trying to use ingress approach flag instead.");

                cnvLane.setApproach(laneData.getIngressApproachExists());

            } else if (dirBitString == 2) // b01 ingress (incorrect on easymile should be flipped)
            {
                //log.debug("Found approach lane: " + Integer.toString(laneData.getLaneId()));
                cnvLane.setApproach(true);

            } else if (dirBitString == 1) { // b10 egress (incorrect on easymile should be flipped)

                //log.debug("Found approach lane: " + Integer.toString(laneData.getLaneId()));
                cnvLane.setApproach(false);

            } else { // b11 pedestrian
                //log.debug("Found pedestrian lane: " + Integer.toString(laneData.getLaneId()));
            }

            cnvLane.setAttributes(0);


            if (data.getIntersectionGeometry().getLaneWidthExists()) {
                cnvLane.setWidth((int)(data.getIntersectionGeometry().getLaneWidth() * CM_PER_M));
            }

            if (laneData.getNodeList().getChoice() == NodeListXY.NODE_SET_XY) {
                for (NodeXY nodeBase : laneData.getNodeList().getNodes().getNodeSetXy()) {
                    NodeOffsetPointXY delta = nodeBase.getDelta();
                    switch (delta.getChoice()) {
                        case NodeOffsetPointXY.NODE_LATLON: // If the node is a lat/lon value
                            Location node = new Location(delta.getLatitude(),
                                    delta.getLongitude());
                            cnvLane.addNodeLatLon(node);
                            break;
                        default: // All other choices result in the same response
                            addNodeOffset(cnvLane, refPoint, delta.getX(), delta.getY());
                            break;
                    }
                }
            }

            laneList.add(cnvLane);
        }
        map.setLanes(laneList);

        return map;
    }

    /**
     * Convert IntersectionData instance into old Glidepath app object
     * @param data The {@class IntersectionData} instance to be converted
     * @return The SPAT data from input converted into a Glidepath formatted object
     */
    static protected SpatMessage convertSpatMessage(IntersectionData data) {
        IntersectionState state = data.getIntersectionState();
        if (state == null) {
            throw new IllegalArgumentException("convertSpatMessage called with null spat");
        }
        SpatMessage cnvSpat = new SpatMessage();

        cnvSpat.setContentVersion(state.getRevision());
        cnvSpat.setIntersectionId(state.getId().getId());

        cnvSpat.setStatus(state.getStatus().getIntersectionStatusObject());
        cnvSpat.setTimeStamp(DateTime.now()); // TODO: Improve estimation of data age

        List<Movement> movements = new ArrayList<>();
        for (MovementState movementData : state.getMovementList()) {
            Movement m = new Movement();

            for (GenericLane lane : data.getIntersectionGeometry().getLaneList()) {
                // Detect based on connectsTo
                if (lane.getConnectsToExists()) {
                    for (j2735_msgs.Connection connectsTo : lane.getConnectToList()) {
                        if (connectsTo.getSignalGroupExists()) {
                            if (connectsTo.getSignalGroup() == movementData.getSignalGroup()) {
                                    
                                // TODO Bad assumption: We are assuming that if there is any connection with the same signal group we can assume there is a straight path
                                    // TODO we still need to have this check but the data is not available in the current map messages
                                    // && connectsTo.getConnectingLane().getManeuverExists()
                                    // && (connectsTo.getConnectingLane().getManeuver().getAllowedManeuvers() | j2735_msgs.AllowedManeuvers.STRAIGHT) > 0) {
                                LaneSet lanes = new LaneSet(lane.getLaneId(), 0x01); // TODO: Detect maneuvers other
                                                                                     // than
                                                                                     // straight
                                m.addLaneSet(lanes);
                                break;
                            }
                        }
                    }
                }
            }

            if (movementData.getMovementEventList().size() > 0) {
                Queue<MovementEvent> sortedEvents = new PriorityQueue<>(
                    (MovementEvent m1, MovementEvent m2) -> {
                        if (!m1.getTimingExists())
                            return 1; // Put events without timing at the end
                        if (!m2.getTimingExists()) 
                            return -1; // Put events without timing at the end
                        return (int) (m1.getTiming().getMinEndTime() - m2.getTiming().getMinEndTime()); // Use the non-optional minEndTiming to sort events
                    }); 

                    sortedEvents.addAll(movementData.getMovementEventList()); // Sort the movement events by minEndTime

                    MovementEvent earliestEvent = sortedEvents.peek();
                    if (earliestEvent.getTimingExists() && earliestEvent.getTiming().getMaxEndTimeExists()) {
                        DateTime dt = DateTime.now();
                        long millisOfDay = dt.getMillisOfDay();
                        long millisToHourStart = dt.getHourOfDay() * 3600000L;
                        double secondInHour = (double)(millisOfDay - millisToHourStart) / 1000.0;
                        m.setMaxTimeRemaining(Math.max(0.0, earliestEvent.getTiming().getMaxEndTime() - secondInHour));
                        m.setMinTimeRemaining(Math.max(0.0, earliestEvent.getTiming().getMinEndTime() - secondInHour));
                        // Check if lights should always be green
                        if (ignoreLights) {
                            m.setMaxTimeRemaining(900.0);
                            m.setMinTimeRemaining(900.0);
                        }
                    }
                //TODO move this statment log.warn("Empty movement event list in spat for intersection id: " + state.getId().getId());
                
                int phase = sortedEvents.peek().getEventState().getMovementPhaseState();
                switch(phase) {
                    case j2735_msgs.MovementPhaseState.PROTECTED_MOVEMENT_ALLOWED: // Green light
                        m.setCurrentState(0x00000001);
                        break;
                    case j2735_msgs.MovementPhaseState.PERMISSIVE_MOVEMENT_ALLOWED: // Green light
                        m.setCurrentState(0x00000001);
                        break;
                    case j2735_msgs.MovementPhaseState.PROTECTED_CLEARANCE: // Yellow
                        m.setCurrentState(0x00000002);
                        break;
                    case j2735_msgs.MovementPhaseState.PERMISSIVE_CLEARANCE: // Yellow
                        m.setCurrentState(0x00000002);
                        break;
                    case j2735_msgs.MovementPhaseState.STOP_AND_REMAIN: // Red light
                        m.setCurrentState(0x00000004);
                        break;
                    default:
                        //log.warn("Unsupported signal phase: " + phase);
                        break;
                }
                // Check if lights should always be green
                if (ignoreLights) {
                    m.setCurrentState(0x00000001);
                }
            }

            movements.add(m);
        }
        cnvSpat.setMovements(movements);

        return cnvSpat;
    }

    private void setPlan(List<Node> plan, double startingDowntrack) {
        synchronized (currentPlan) {
            currentPlan.set(plan);
            planStartingDowntrack.set(startingDowntrack);
        }
    }

    @Override
    public void triggerNewPlan(final boolean availability) {
        log.info("Trying to request replan with availability: " + availability);
        if (replanning.compareAndSet(false, true)) {

            // Just return if the availability is false
            if (!availability) {
                setAvailability(availability);
                pluginServiceLocator.getArbitratorService().requestNewPlan();
                replanning.set(false);
                return;
            }

            // Generate plan in new thread
            Thread planningThread = new Thread(new Runnable() {

                @Override
                public void run() {
                    log.info("Attempting Ead Plan");
                    // Planning
                    double currentDowntrack = pluginServiceLocator.getRouteService().getCurrentDowntrackDistance();
                    double speedLimit = pluginServiceLocator.getRouteService().getSpeedLimitAtLocation(currentDowntrack).getLimit();
                    DataElementHolder state = null;
                    List<Node> eadResult = null;
                    
                    for (int i = 0; i < MAX_PLAN_RETRIES; i++) {
                        state = getCurrentStateData(speedLimit);
        
                        eadResult = generatePlan(state);
                        if (eadResult != null) {
                            break;
                        }
                    }
                    
                    // Store the current plan
                    DoubleDataElement startTime = (DoubleDataElement) state.get(DataElementKey.PLANNING_START_TIME);
                    DoubleDataElement startDowntrack = (DoubleDataElement) state.get(DataElementKey.PLANNING_START_DOWNTRACK);
        
                    setPlan(eadResult, startDowntrack.value());
                    
                    if (eadResult == null) {
                        log.warn("Ead result was null requesting plan with no availability");
                        setAvailability(false);
                        pluginServiceLocator.getArbitratorService().requestNewPlan();
                        replanning.set(false);
                        return;
                    }
        
                    log.info("EadAStar result is path of size: " + eadResult.size());
                    //log.info("EadAStar Num ANA iterations: " + ANAStarSolver.iterationCount);
                    // Set the new plan as the current plan for collision checker
                    collisionChecker.setHostPlan(eadResult, startTime.value(), startDowntrack.value());
        
                    log.info("Planning Successful Requesting replan with availability: " + availability);
                    setAvailability(availability);
                    pluginServiceLocator.getArbitratorService().requestNewPlan();
                }

            });

            planningThread.start();

            
        }
    }

    private List<Node> generatePlan(DataElementHolder state) {
        // Try 3 times to plan. With updated state data each time if needed. We will accept the first completed plan
        log.info("Requesting AStar plan with intersections: " + intersections.toString());

        try {
            // TODO remove this print after NCV handling is stable
            if (appConfig.getBooleanValue("ead.handleNCV")) {
                synchronized (collisionChecker) {
                    log.info("EAD", "NCV at start of planning");
                    for (Entry<Integer, List<RoutePointStamped>> prediction: ((ObjectCollisionChecker) collisionChecker).trackedLaneObjectsPredictions.entrySet()) {
                        String predictionText = "Prediction for id: " + prediction.getKey() + "\n";
                        for (RoutePointStamped p: prediction.getValue()) {
                            predictionText += p.toString() + "\n";
                        }
                        log.info("EAD", predictionText);
                    }
                    log.info("EAD", "Done NCV list");
                }
            }
            return glidepathTrajectory.plan(state); // TODO do we really need the trajectory object. It's purpose is 99% filled by the plugin
        } catch (Exception e) {
            log.error("Glidepath trajectory planning threw exception!", e);
        }
        return null;
    }

    /**
     * Callback receiver for new intersection data from {@link V2IService}
     */
    private void handleNewIntersectionData(List<IntersectionData> data) {
        synchronized (intersections) {
            // Get current intersection if available to compare for phase change
            Integer currentIntId = null;
            if (glidepathTrajectory != null) {
                List<gov.dot.fhwa.saxton.carma.signal_plugin.asd.IntersectionData> trackedIntersections = glidepathTrajectory.getSortedIntersections();
                if (trackedIntersections.size() > 0) {
                    currentIntId = trackedIntersections.get(0).getIntersectionId();
                }
            }

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
                        log.warn("Intersection could not be processed because it has no state. Id: " + old.getIntersectionId());
                        continue;
                    }
                    
                    if (currentIntId != null && currentIntId != datum.getIntersectionId()) {
                        log.debug("Ignoring phase check for future intersection: " + datum.getIntersectionId());
                        continue;
                    }
                    phaseChangeCheckLoop: for (MovementState oldMov : oldIntersectionState.getMovementList()) {
                        for (MovementState newMov : datum.getIntersectionState().getMovementList()) {
                            if (oldMov.getSignalGroup() == newMov.getSignalGroup()) {
                                // If we find a shared movement, check the movement events for sameness

                                // ASSUMPTION: Phase states within a movement will always be reported in the same order
                                // There is no ID for an individual movement event within a movement, so I can only 
                                // compare different messages based on positional similarity.
                                for (int i = 0; i < oldMov.getMovementEventList().size(); i++) {
                                    if (i >= newMov.getMovementEventList().size()
                                            || (oldMov.getMovementEventList().get(i)
                                                    .getEventState().getMovementPhaseState() != newMov.getMovementEventList().get(i)
                                                    .getEventState().getMovementPhaseState() && newMov.getMovementEventList().get(i)
                                                    .getEventState().getMovementPhaseState() != MovementPhaseState.PROTECTED_CLEARANCE)) { // Do not replan on transition to yellow
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
            boolean deletedIntersection = intersections.entrySet().removeIf(entry -> !foundIds.contains(entry.getKey()));
            // Trigger new plan on phase change
            double dtsb = computeDtsb();
            boolean onMap = checkIntersectionMaps(dtsb);
            TwistStamped curTwist = curVel.get();
            double curSpeed = defaultSpeedLimit;
            if (curTwist != null) {
                curSpeed = curTwist.getTwist().getLinear().getX();
            }
            double maxStoppingDistance = (0.5 * curSpeed * curSpeed) / defaultAccel;
            double MIN_REPLANNING_DTSB = Math.max(twiceAcceptableStopDistance, 1.0 + maxStoppingDistance);

            if (!stoppedAtLight() && (phaseChanged || newIntersection || deletedIntersection) && onMap && dtsb > MIN_REPLANNING_DTSB) {
                log.info("Requesting new plan with causes - PhaseChanged: " + phaseChanged 
                    + " NewIntersection: " + newIntersection 
                    + " deletedIntersection: " + deletedIntersection
                    + " onMap: " + onMap);
                triggerNewPlan(true);
            }
        }
    }

    /**
     * Compute the DTSB value if we're on a known MAP message
     * 
     * @return Integer.MAX_VALUE if DTSB check fails for any reason
     */
    private double computeDtsb() {
        if (curPos.get() == null) { // NULL will only be present at startup and will never be assigned making this check thread safe.
            return MAX_DTSB; 
        }
        if (glidepathTrajectory == null) {
            return MAX_DTSB;
        }

        try {
            double dtsb = glidepathTrajectory.updateIntersections(convertIntersections(intersections), curPos.get());
            updateUISignals();
            if (dtsb >= MAX_DTSB) {
                log.debug("DTSB computation failed! Returning MAX_DTSB");
                return MAX_DTSB;
            }
            return dtsb;
        } catch (Exception e) {
            log.debug("DTSB computation failed!", e);
        }

        return MAX_DTSB;
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
        signalMsg.setIntersectionId((short)intersection.getIntersectionId());
        signalMsg.setLaneId((short)intersection.getLaneId());
        signalMsg.setRemainingDistance((float)intersection.getDtsb());
        signalMsg.setRemainingTime((short)intersection.getTimeToNextPhase());

        log.debug("UI Intersection: " + intersection.getIntersectionId() + " dtsb: " + intersection.getDtsb());
        
        if (intersection.getCurrentPhase() == null) {
            return signalMsg;
        }

        switch(intersection.getCurrentPhase()) {
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

    private boolean checkIntersectionMaps(double dtsb) {
        return dtsb < MAX_DTSB;
    }
    /**
     * Check to see if we're on a MAP message
     * 
     * @return true, if we're on a MAP message, false O.W.
     */
    private boolean checkIntersectionMaps() {
        double dtsb = computeDtsb();
        log.debug("Computed DTSB: " + dtsb);
        return dtsb < MAX_DTSB;
    }

    @Override
    public void onResume() {
        log.info("TrafficSignalPlugin trying to resume.");
        
        velocitySub.registerOnMessageCallback((msg) -> {
            curVel.set(msg);
            velFilter.addRawDataPoint(msg.getTwist().getLinear().getX());
        });

        defaultSpeedLimit = appConfig.getMaximumSpeed(0.0);
        ead = new EadAStar(collisionChecker);
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

        // Update radar detection
        RoadwayEnvironment lastMsg = obstacleSub.getLastMessage();
        if (lastMsg != null) {
            collisionChecker.updateObjects(obstacleSub.getLastMessage().getRoadwayObstacles());
        }
        
        // Update vehicle position with most recent transform
        updateCurrentLocation();
        
        // Update state variables for stopping case
        evaluateStatesForStopping();
        
        long tsEnd = System.currentTimeMillis();
        long sleepDuration = Math.max(LOOP_PERIOD - (tsEnd - tsStart), 0);
        Thread.sleep(sleepDuration);
    }

    /**
     * Helper function to evaluate state variables for the stopping condition
     */
    private void evaluateStatesForStopping() {

        // We were stopped at a red light and now the light is green or will be in a few seconds
        boolean stoppedForLight = stoppedAtLight(); //Only set when inside planTrajectory method.
        boolean phaseIsGreen = checkCurrentPhase(SignalPhase.GREEN);
        boolean phaseIsGreenSoon = phaseIsGreen || checkCurrentPhaseAndRemainingTime(SignalPhase.RED, popupOnRedTime);

        if (stoppedForLight && phaseIsGreenSoon) {

            // If we have not requested acknowledgement from the user do it now
            if (awaitingUserConfirmation.compareAndSet(false, true)) {
                // Send user input request to get confirmation to continue
                askUserIfIntersectionIsClear();
                log.info("Requesting user confirmation");
            }
            else{
                log.info("Already waiting for confirmation.");
            }

        } else if (!stoppedForLight && awaitingUserConfirmation.compareAndSet(true,false)){
            // We are no longer waiting at the intersection
            // NOTE: stoppedForLight is only re-evaluated in planTrajectory(), so it will it will only be set from true to false at that time.
            // When stoppedForLight is no longer true (meaning vehicle is on the move again) AND awaitingUserConfirmation is true at the time,
            // then set awaitingUserConfirmation back to false and resume operation.
            log.info("Resuming operation after stop");
        }

    }
    /**
     * Helper function which gets the current lat/lon position of the vehicle's front bumper
     */
    private void updateCurrentLocation() {
        Transform earthToHostVehicle = getTransform("earth", "vehicle_front", Time.fromMillis(0));
        if (earthToHostVehicle == null) {
            log.warn("Failed to get transform from vehicle_front to earth");
            return;
        }
        Vector3 transInECEF = earthToHostVehicle.getTranslation();
        Point3D hostVehicleInECEF = new Point3D(transInECEF.getX(), transInECEF.getY(), transInECEF.getZ());
        gov.dot.fhwa.saxton.carma.geometry.geodesic.Location carmaLocation = gcc.cartesian2Geodesic(hostVehicleInECEF, Transform.identity());
        curPos.set(new Location(carmaLocation.getLatitude(), carmaLocation.getLongitude()));

        if (checkIntersectionMaps()) {
            if (involvedInControl.compareAndSet(false, true)) {
                log.info("On map and replanning");
                triggerNewPlan(true);
            }
        } else if (involvedInControl.compareAndSet(true, false)) {
            log.info("Off map requesting replan");

            triggerNewPlan(false);
        }
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
        LongitudinalManeuver currentLongitudinalManeuver = (LongitudinalManeuver) pluginServiceLocator.getArbitratorService().getCurrentlyExecutingManeuver(ManeuverType.LONGITUDINAL);
        if (currentLongitudinalManeuver == null) {
            return false; // Can't be stopped at a light if we are not under automated control or in a complex maneuver which we would not have planned
        }
        
        return currentLongitudinalManeuver.getPlanner().equals(this) 
            && currentLongitudinalManeuver instanceof SteadySpeed
            && Math.abs(currentLongitudinalManeuver.getTargetSpeed()) < ZERO_SPEED_NOISE;
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
        return !sortedIntersections.isEmpty() && sortedIntersections.get(0).getCurrentPhase() == phase;
    }

    /**
     * Helper function returns true if the current phase of the nearest intersection is equal to the requested phase
     * and there is less than or equal time remaining to the specified maxTimeRemaining
     * 
     * @param phase The phase to check against
     * @param maxTimeRemaining The max time allowed to be remaining in this phase for the function to return true
     * 
     * @return True if nearest intersection phase is equal to current phase and has less than or equal time remaining to maxTimeRemaining
     */
    private boolean checkCurrentPhaseAndRemainingTime(SignalPhase phase, double maxTimeRemaining) {
        List<gov.dot.fhwa.saxton.carma.signal_plugin.asd.IntersectionData> sortedIntersections = glidepathTrajectory.getSortedIntersections();
        return !sortedIntersections.isEmpty() 
            && sortedIntersections.get(0).getCurrentPhase() == phase
            && sortedIntersections.get(0).getTimeToNextPhase() <= maxTimeRemaining;
    }

    @Override
    public void onSuspend() {
    	velocitySub.close();
    	obstacleSub.close();
        log.info("SignalPlugin has been suspended.");
    }

    @Override
    public void onTerminate() {
    }

    static protected List<gov.dot.fhwa.saxton.carma.signal_plugin.asd.IntersectionData> convertIntersections(
            Map<Integer, IntersectionData> data) {
        List<gov.dot.fhwa.saxton.carma.signal_plugin.asd.IntersectionData> out = new ArrayList<>();
        synchronized (data) {
            for (IntersectionData datum : data.values()) {
                gov.dot.fhwa.saxton.carma.signal_plugin.asd.IntersectionData converted = new gov.dot.fhwa.saxton.carma.signal_plugin.asd.IntersectionData();
                converted.setMap(convertMapMessage(datum));
                converted.setIntersectionId(converted.getMap().getIntersectionId());
                
                if (datum.getIntersectionState() != null) {
                    converted.setSpat(convertSpatMessage(datum));
                    //log.debug("Converted map message with no spat");
                }
            
                out.add(converted);
            }
        }

        return out;
    }

    /**
     * NOTE: This function assumes any requests to change the planning process (higher priority / longer trajectories) will be granted
     */
    @Override
    public TrajectoryPlanningResponse planTrajectory(Trajectory traj, double expectedStartSpeed) {
        log.info("Entered planTrajectory");

        // If we are stopped at a light we must stay stopped until user confirmation.
        boolean stoppedForLight = stoppedAtLight();
        boolean phaseIsGreen = checkCurrentPhase(SignalPhase.GREEN);
        if (stoppedForLight && !phaseIsGreen && awaitingUserConfirmation.get()) {
            // CHECK PLANNING PRIORITY
            if (!traj.getLongitudinalManeuvers().isEmpty()) {
                TrajectoryPlanningResponse tpr = new TrajectoryPlanningResponse();
                tpr.requestHigherPriority();
                return tpr;
            }
            SteadySpeed steadySpeed = new SteadySpeed(this);
            steadySpeed.setMaxAccel(pluginManeuverInputs.getMaxAccelLimit() * 2.0); // TODO determine if having twice the max accel is really ok

            steadySpeed.setSpeeds(0.0, 0.0);
            pluginManeuverPlanner.planManeuver(steadySpeed,
                    traj.getStartLocation(), traj.getEndLocation());
            
            traj.addManeuver(steadySpeed);
            replanning.set(false);
            return new TrajectoryPlanningResponse();
        }

        // CHECK PLANNING PRIORITY
        if (!traj.getLongitudinalManeuvers().isEmpty()) {
            TrajectoryPlanningResponse tpr = new TrajectoryPlanningResponse();
            tpr.requestHigherPriority();
            return tpr;
        }

        log.info("Granted highest planning priority");

        // See if valid plan is available
        List<Node> eadResult = currentPlan.get();

        if (eadResult == null) {
            log.warn("PlanTrajectory: Ead result is null");
            replanning.set(false);
            return new TrajectoryPlanningResponse();
        } else {
            log.info("PlanTrajectory: Ead result is path of size: " + eadResult.size());
        }

        // CHECK TRAJECTORY LENGTH

        // DTSB computation successful, check to see if we can plan up to stop bar
        double planLength = eadResult.get(eadResult.size() - 1).getDistanceAsDouble();
        if (traj.getStartLocation() + ( planLength / 0.7 ) > traj.getEndLocation()) {
            // Not enough distance to allow for proper glidepath execution
            TrajectoryPlanningResponse tpr = new TrajectoryPlanningResponse();
            tpr.requestLongerTrajectory(traj.getStartLocation() + (planLength / 0.69)); // allow for some extra slack
            return tpr;
        }

        log.info("DTSB within traj");



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
        // TODO we should get the starting distance as current downtrack when starting to plan
        double startDist;
        synchronized (currentPlan) {
            startDist = planStartingDowntrack.get(); // TODO this change is not thread safe
        }
        Node prev = null;
        double prevManeuverEndDist = 0;
        boolean firstManeuver = true;
        for (Node cur : eadResult) {
            if (prev == null) {
                prev = cur;
                prevManeuverEndDist = startDist + prev.getDistanceAsDouble();
            } else {

                double maneuverEndDist = prevManeuverEndDist + (cur.getDistanceAsDouble() - prev.getDistanceAsDouble());
                
                if (maneuverEndDist < traj.getStartLocation()) {
                    prev = cur;
                    prevManeuverEndDist = maneuverEndDist;
                    continue;
                    
                } else if (firstManeuver) {
                    prevManeuverEndDist = traj.getStartLocation();
                    firstManeuver = false;
                }

                if (Math
                        .abs(prev.getSpeedAsDouble() - cur.getSpeedAsDouble()) < speedCommandQuantizationFactor) {
                    SteadySpeed steadySpeed = new SteadySpeed(this);
                    steadySpeed.setMaxAccel(pluginManeuverInputs.getMaxAccelLimit() * 2.0); // TODO determine if having twice the max accel is really ok
                    
                    if (cur.getSpeedAsDouble() > speedCommandQuantizationFactor) {
                        steadySpeed.setSpeeds(cur.getSpeedAsDouble(), cur.getSpeedAsDouble());
                        pluginManeuverPlanner.planManeuver(steadySpeed,
                                prevManeuverEndDist, maneuverEndDist);
                    } else {
                        // We're coming to a stop, so plan an indefinite length stop maneuver, to be
                        // overridden by replan later
                        steadySpeed.setSpeeds(0.0, 0.0);
                        pluginManeuverPlanner.planManeuver(steadySpeed,
                                prevManeuverEndDist, traj.getEndLocation());
                        
                        traj.addManeuver(steadySpeed);
                        break;
                    }

                    traj.addManeuver(steadySpeed);
                } else if (prev.getSpeedAsDouble() < cur.getSpeedAsDouble()) {
                    SpeedUp speedUp = new SpeedUp(this);
                    speedUp.setMaxAccel(pluginManeuverInputs.getMaxAccelLimit());
                    speedUp.setSpeeds(prev.getSpeedAsDouble(), cur.getSpeedAsDouble());
                    pluginManeuverPlanner.planManeuver(speedUp,
                            prevManeuverEndDist, maneuverEndDist);
                    traj.addManeuver(speedUp);
                } else {
                    SlowDown slowDown = new SlowDown(this);
                    slowDown.setMaxAccel(pluginManeuverInputs.getMaxAccelLimit());
                    slowDown.setSpeeds(prev.getSpeedAsDouble(), cur.getSpeedAsDouble());
                    pluginManeuverPlanner.planManeuver(slowDown,
                            prevManeuverEndDist, maneuverEndDist);
                    traj.addManeuver(slowDown);
                } 

                prevManeuverEndDist = maneuverEndDist;
                prev = cur;
            }
        }
        
        log.info("Planning complete");
        log.info("Added Maneuvers: " + traj.getLongitudinalManeuvers().toString());
        
        replanning.set(false);
        return new TrajectoryPlanningResponse();
    }

    private DataElementHolder getCurrentStateData(double operatingSpeed) {
        DataElementHolder state = new DataElementHolder();
        DoubleDataElement curSpeedElement;
        DoubleDataElement curAccelElement;
        DoubleDataElement operSpeedElem;
        DoubleDataElement vehicleLat;
        DoubleDataElement vehicleLon;
        DoubleDataElement planningStartTime;
        DoubleDataElement planningStartDowntrack;
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
        operSpeedElem = new DoubleDataElement(operatingSpeed * operSpeedScalingFactor); // NOTE: This field is used as the speed limit for Ead algorithm and does not support variables speed limits in a route
        state.put(DataElementKey.OPERATING_SPEED, operSpeedElem);

        Location curLoc = curPos.get();
        vehicleLat = new DoubleDataElement(curLoc.lat());
        vehicleLon = new DoubleDataElement(curLoc.lon());
        state.put(DataElementKey.LATITUDE, vehicleLat);
        state.put(DataElementKey.LONGITUDE, vehicleLon);

        IntersectionCollection ic = new IntersectionCollection();
        ic.setIntersections(convertIntersections(intersections));
        icde = new IntersectionCollectionDataElement(ic);
        state.put(DataElementKey.INTERSECTION_COLLECTION, icde);

        planningStartTime = new DoubleDataElement(pluginServiceLocator.getTimeProvider().getCurrentTimeSeconds());
        planningStartDowntrack = new DoubleDataElement(pluginServiceLocator.getRouteService().getCurrentDowntrackDistance());
        state.put(DataElementKey.PLANNING_START_TIME, planningStartTime);
        state.put(DataElementKey.PLANNING_START_DOWNTRACK, planningStartDowntrack);

        return state;
    }

    private Transform getTransform(String parentFrame, String childFrame, Time stamp) {
	    GetTransformRequest request = getTransformClient.newMessage();
	    request.setParentFrame(parentFrame);
	    request.setChildFrame(childFrame);
	    request.setStamp(stamp);

	    final GetTransformResponse[] response = new GetTransformResponse[1];
	    final boolean[] gotTransform = {false};

	    getTransformClient.call(request,
		    new OnServiceResponseCallback<GetTransformResponse>() {

			    @Override
			    public void onSuccess(GetTransformResponse msg) {
			    	if (msg.getErrorStatus() == GetTransformResponse.NO_ERROR
					    || msg.getErrorStatus() == GetTransformResponse.COULD_NOT_EXTRAPOLATE) {
					    response[0] = msg;
					    gotTransform[0] = true;
				    } else {
			    		log.warn("TRANSFORM", "Request for transform ParentFrame: " + request.getParentFrame() +
						    " ChildFrame: " + request.getChildFrame() + " returned ErrorCode: " + msg.getErrorStatus());
				    }
			    }

			    @Override
			    public void onFailure(Exception e) {
                    log.error("get_transform service call failed", e);
				    // TODO how to handle this case?
			    }

		    });

	    if(gotTransform[0]) {
	    	return Transform.fromTransformMessage(response[0].getTransform().getTransform());
	    } else {
	    	return null;
	    }
    }
}
