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
import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicReference;

import org.joda.time.DateTime;

import cav_msgs.Connection;
import cav_msgs.GenericLane;
import cav_msgs.IntersectionState;
import cav_msgs.MovementState;
import cav_msgs.NodeListXY;
import cav_msgs.NodeOffsetPointXY;
import cav_msgs.NodeXY;
import cav_msgs.Position3D;
import geometry_msgs.TwistStamped;
import gov.dot.fhwa.saxton.carma.guidance.arbitrator.TrajectoryPlanningResponse;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.SlowDown;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.SpeedUp;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.SteadySpeed;
import gov.dot.fhwa.saxton.carma.guidance.plugins.AbstractPlugin;
import gov.dot.fhwa.saxton.carma.guidance.plugins.IStrategicPlugin;
import gov.dot.fhwa.saxton.carma.guidance.plugins.PluginServiceLocator;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.ISubscriber;
import gov.dot.fhwa.saxton.carma.guidance.trajectory.Trajectory;
import gov.dot.fhwa.saxton.carma.guidance.util.IntersectionData;
import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.DataElementHolder;
import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.DataElementKey;
import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.DoubleDataElement;
import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.GlidepathAppConfig;
import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.IntersectionCollectionDataElement;
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

/**
 * Top level class in the Traffic Signal Plugin that does trajectory planning through
 * signalized intersections. This is a port of the functionality developed under
 * the STOL I contract TO 17 and STOL II contract TO 13, Glidepath project.
 */
public class TrafficSignalPlugin extends AbstractPlugin implements IStrategicPlugin {

    private ISubscriber<NavSatFix> gpsSub;
    private ISubscriber<TwistStamped> velocitySub;
    private Map<Integer, IntersectionData> intersections = Collections
            .synchronizedMap(new HashMap<Integer, IntersectionData>());
    private AtomicReference<NavSatFix> curPos = new AtomicReference<>();
    private AtomicReference<TwistStamped> curVel = new AtomicReference<>();
    private gov.dot.fhwa.saxton.carma.signal_plugin.ead.Trajectory glidepathTrajectory;
    private PolyHoloA velFilter = new PolyHoloA();
    private EadAStar ead;
    private double operSpeedScalingFactor = 1.0;
    private double speedCommandQuantizationFactor = 0.1;

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

        ead = new EadAStar();
        try {
            glidepathTrajectory = new gov.dot.fhwa.saxton.carma.signal_plugin.ead.Trajectory(ead);
        } catch (Exception e) {
            log.error("Glidepath unable to initialize EAD algorithm!!!", e);
            setAvailability(false);
            return;
        }
        
        // log the key params here
        pluginServiceLocator.getV2IService().registerV2IDataCallback(this::handleNewIntersectionData);
        setAvailability(false);
        gpsSub = pluginServiceLocator.getPubSubService().getSubscriberForTopic("nav_sat_fix", NavSatFix._TYPE);
        gpsSub.registerOnMessageCallback((msg) -> {
            curPos.set(msg);
            if (checkIntersectionMaps()) {
                triggerNewPlan();
            }
        });

        velocitySub = pluginServiceLocator.getPubSubService().getSubscriberForTopic("velocity", TwistStamped._TYPE);
        velocitySub.registerOnMessageCallback((msg) -> {
            curVel.set(msg);
            velFilter.addRawDataPoint(msg.getTwist().getLinear().getX());
        });

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
     * Trigger a new plan from arbitrator to accommodate for Glidepath planning
     */
    private void triggerNewPlan() {
        setAvailability(true);
        pluginServiceLocator.getArbitratorService().requestNewPlan();
    }

    /**
     * Callback receiver for new intersection data from {@link V2IService}
     */
    private void handleNewIntersectionData(List<IntersectionData> data) {
        synchronized (intersections) {
            boolean phaseChanged = false;
            boolean newIntersection = false;
            for (IntersectionData datum : data) {
                if (!intersections.containsKey(datum.getIntersectionId())) {
                    intersections.put(datum.getIntersectionId(), datum);
                    newIntersection = true;
                } else {
                    // If it is in the list, check to see if this has changed the phase
                    IntersectionData old = intersections.get(datum.getIntersectionId());

                    /**
                     * This code is looking like it's O(bad) asymptotic complexity in the worst
                     * case, but I'm not sure it's an issue as we're unlikely to experience very
                     * large numbers of movements in a single intersection. But in the event
                     * performance is an issue this might be a likely culprit to investigate.
                     * 
                     * -KR
                     */

                    // Walk through all the movements to compare
                    phaseChangeCheckLoop: for (MovementState oldMov : old.getIntersectionState().getStates()
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

            boolean deletedIntersection = false;
            for (Integer id : intersections.keySet()) {
                boolean found = false;
                for (IntersectionData datum : data) {
                    if (datum.getIntersectionId() == id) {
                        intersections.put(datum.getIntersectionId(), datum);
                        found = true;
                    }
                }

                if (!found) {
                    intersections.remove(id);
                    deletedIntersection = true;
                }
            }

            if ((phaseChanged || newIntersection || deletedIntersection) && checkIntersectionMaps()) {
                triggerNewPlan();
            }
        }
    }

    /**
     * Compute the DTSB value if we're on a known MAP message
     * 
     * @return -1 if DTSB check fails for any reason, Double.MAX_VALUE if we're not on a MAP message
     */
    private double computeDtsb() {
        Location curLoc = new Location(curPos.get().getLatitude(), curPos.get().getLongitude());

        try {
            return glidepathTrajectory.updateIntersections(convertIntersections(intersections), curLoc);
        } catch (Exception e) {
            log.warn("DTSB computation failed!");
        }

        return -1;
    }

    /**
     * Check to see if we're on a MAP message
     * 
     * @return true, if we're on a MAP message, false O.W.
     */
    private boolean checkIntersectionMaps() {
        double dtsb = computeDtsb();
        return dtsb > 0 && dtsb < Double.MAX_VALUE;
    }

    @Override
    public void onResume() {
        log.info("SignalPlugin has resumed.");
        pluginServiceLocator.getV2IService().registerV2IDataCallback(this::handleNewIntersectionData);
    }

    @Override
    public void loop() throws InterruptedException {
        long tsStart = System.currentTimeMillis();

        long tsEnd = System.currentTimeMillis();
        long sleepDuration = Math.max(100 - (tsEnd - tsStart), 0);
        Thread.sleep(sleepDuration);
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

        // CHECK PLANNING PRIORITY
        if (!traj.getLongitudinalManeuvers().isEmpty()) {
                TrajectoryPlanningResponse tpr = new TrajectoryPlanningResponse();
                tpr.requestHigherPriority();
                return tpr;
        }

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

        try {
            glidepathTrajectory.getSpeedCommand(state);
        } catch (Exception e) {
            log.error("Glidepath trajectory planning threw exception!", e);
            TrajectoryPlanningResponse tpr = new TrajectoryPlanningResponse();
            tpr.requestHigherPriority(); // indicate generic failure
            return tpr;
        }

        // GET NODES OUT OF EADASTAR
        List<Node> eadResult = ead.getCurrentPath();

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

        setAvailability(false);
        return new TrajectoryPlanningResponse();
    }
}
