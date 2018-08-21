package gov.dot.fhwa.saxton.carma.signal_plugin;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicReference;

import org.joda.time.DateTime;

import cav_msgs.Connection;
import cav_msgs.ConnectsToList;
import cav_msgs.GenericLane;
import cav_msgs.IntersectionState;
import cav_msgs.MovementState;
import cav_msgs.NodeListXY;
import cav_msgs.NodeOffsetPointXY;
import cav_msgs.NodeXY;
import cav_msgs.Position3D;
import cav_msgs.SPAT;
import geometry_msgs.TwistStamped;
import gov.dot.fhwa.saxton.carma.geometry.cartesian.Point3D;
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
import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.IntersectionCollectionDataElement;
import gov.dot.fhwa.saxton.carma.signal_plugin.asd.IntersectionCollection;
import gov.dot.fhwa.saxton.carma.signal_plugin.asd.Lane;
import gov.dot.fhwa.saxton.carma.signal_plugin.asd.Location;
import gov.dot.fhwa.saxton.carma.signal_plugin.asd.map.MapMessage;
import gov.dot.fhwa.saxton.carma.signal_plugin.asd.spat.LaneSet;
import gov.dot.fhwa.saxton.carma.signal_plugin.asd.spat.Movement;
import gov.dot.fhwa.saxton.carma.signal_plugin.asd.spat.SpatMessage;
import gov.dot.fhwa.saxton.carma.signal_plugin.ead.EadAStar;
import gov.dot.fhwa.saxton.carma.signal_plugin.ead.IEad;
import gov.dot.fhwa.saxton.carma.signal_plugin.ead.trajectorytree.Node;
import gov.dot.fhwa.saxton.carma.signal_plugin.filter.PolyHoloA;
import sensor_msgs.NavSatFix;

/**
 * Top level class in the Signal Plugin that does trajectory planning through
 * signalized intersections. This is a port of the functionality developed under
 * the STOL I contract TO 17 and STOL II contract TO 13, Glidepath project.
 */
public class SignalPlugin extends AbstractPlugin implements IStrategicPlugin {

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

    public SignalPlugin(PluginServiceLocator psl) {
        super(psl);
        version.setName("Signal Plugin");
        version.setMajorRevision(1);
        version.setIntermediateRevision(0);
        version.setMinorRevision(0);
    }

    // TODO: this is a skeleton! Unless otherwise noted, all comments below are PDL
    // that need to be expanded to working code

    @Override
    public void onInitialize() {
        // load params

        log.info("STARTUP", "SignalPlugin has been initialize.");
        // log the key params here
        pluginServiceLocator.getV2IService().registerV2IDataCallback(this::handleNewIntersectionData);
        setAvailability(false);
        gpsSub = pluginServiceLocator.getPubSubService().getSubscriberForTopic("~nav_sat_fix", NavSatFix._TYPE);
        gpsSub.registerOnMessageCallback((msg) -> {
            curPos.set(msg);
            if (checkIntersectionMaps()) {
                triggerNewPlan();
            }
        });

        velocitySub = pluginServiceLocator.getPubSubService().getSubscriberForTopic("~velocity", TwistStamped._TYPE);
        velocitySub.registerOnMessageCallback((msg) -> {
            curVel.set(msg);
            velFilter.addRawDataPoint(msg.getTwist().getLinear().getX());
        });

        ead = new EadAStar();
        try {
            glidepathTrajectory = new gov.dot.fhwa.saxton.carma.signal_plugin.ead.Trajectory(ead);
        } catch (Exception e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
    }

    private void addNodeOffset(Lane lane, Location ref, float x, float y) {
        lane.addNodeCm(ref, (int) (x * 100), (int) (y * 100));
    }

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

    private SpatMessage convertSpatMessage(IntersectionData data) {
        IntersectionState state = data.getIntersectionState();
        SpatMessage cnvSpat = new SpatMessage();

        cnvSpat.setContentVersion(state.getRevision());
        if (state.getId().getIdExists()) {
            cnvSpat.setIntersectionId(state.getId().getId());
        } else {
            cnvSpat.setIntersectionId(0);
        }

        cnvSpat.setStatus(state.getStatus().getIntersectionStatusObject());
        cnvSpat.setTimeStamp(DateTime.now()); // TODO: Improve estimation of data age

        List<Movement> movements = new ArrayList<>();
        for (MovementState movementData : state.getStates().getMovementList()) {
            Movement m = new Movement();

            for (GenericLane lane : data.getIntersectionGeometry().getLaneSet().getLaneList()) {
                // Detect based on connectsTo
                if (lane.getConnectsToExists()) {
                    Connection connectsTo = lane.getConnectsTo().getConnectToList();
                    if (connectsTo.getSignalGroupExists()) {
                        if (connectsTo.getSignalGroup() == movementData.getSignalGroup()
                                && connectsTo.getConnectingLane().getManeuverExists()
                                && connectsTo.getConnectingLane().getManeuver().getAllowedManeuvers() == 0) {
                            LaneSet lanes = new LaneSet(lane.getLaneId(), 0x01); // TODO: Detect maneuvers other than
                                                                                 // straight
                            m.addLaneSet(lanes);
                        }
                    }

                }
            }

            movements.add(m);
        }
        cnvSpat.setMovements(movements);

        return cnvSpat;
    }

    private void triggerNewPlan() {
        setAvailability(true);
        pluginServiceLocator.getArbitratorService().requestNewPlan();
    }

    private void handleNewIntersectionData(List<IntersectionData> data) {
        synchronized (intersections) {
            boolean newIntersection = false;
            for (IntersectionData datum : data) {
                if (!intersections.containsKey(datum.getIntersectionId())) {
                    intersections.put(datum.getIntersectionId(), datum);
                    newIntersection = true;
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

            if ((newIntersection || deletedIntersection) && checkIntersectionMaps()) {
                triggerNewPlan();
            }
        }
    }

    private boolean checkIntersectionMaps() {
        for (IntersectionData datum : intersections.values()) {
            MapMessage map = convertMapMessage(datum);
            map.get
        }
        return false;
    }

    @Override
    public void onResume() {
        log.info("SignalPlugin has resumed.");
        pluginServiceLocator.getV2IService().registerV2IDataCallback((data) -> {

        });
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
        }

        // GET NODES OUT OF EADASTAR
        List<Node> eadResult = ead.getCurrentPath();

        // OPTIMIZE NODES
        List<Node> optimizedOutput = new ArrayList<>();

        if (eadResult.size() > 2) {
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
        }

        // CONVERT AND INSERT MANEUVERS
        double startDist = traj.getStartLocation();
        Node prev = null;
        for (Node cur : optimizedOutput) {
            if (prev == null) {
                prev = cur;
            } else {
                if (prev.getSpeedAsDouble() < cur.getSpeedAsDouble()) {
                    SpeedUp speedUp = new SpeedUp(this);
                    speedUp.setSpeeds(prev.getSpeedAsDouble(), cur.getSpeedAsDouble());
                    pluginServiceLocator.getManeuverPlanner().planManeuver(speedUp, startDist + prev.getDistanceAsDouble(), startDist + cur.getDistanceAsDouble());
                    traj.addManeuver(speedUp);
                } else if (prev.getSpeedAsDouble() > cur.getSpeedAsDouble()) {
                    SlowDown slowDown = new SlowDown(this);
                    slowDown.setSpeeds(prev.getSpeedAsDouble(), cur.getSpeedAsDouble());
                    pluginServiceLocator.getManeuverPlanner().planManeuver(slowDown, startDist + prev.getDistanceAsDouble(), startDist + cur.getDistanceAsDouble());
                    traj.addManeuver(slowDown);
                } else if (Math.abs(prev.getSpeedAsDouble() - cur.getSpeedAsDouble()) < speedCommandQuantizationFactor) {
                    SteadySpeed steadySpeed = new SteadySpeed(this);

                    if (cur.getSpeedAsDouble() > speedCommandQuantizationFactor) {
                        steadySpeed.setSpeeds(cur.getSpeedAsDouble(), cur.getSpeedAsDouble());
                        pluginServiceLocator.getManeuverPlanner().planManeuver(steadySpeed, startDist + prev.getDistanceAsDouble(), startDist + cur.getDistanceAsDouble());
                    } else {
                        // We're coming to a stop, so plan an indefinite length stop maneuver, to be overridden by replan later
                        steadySpeed.setSpeeds(0.0, 0.0);
                        pluginServiceLocator.getManeuverPlanner().planManeuver(steadySpeed, startDist + prev.getDistanceAsDouble(), traj.getEndLocation());
                        break;
                    }

                    traj.addManeuver(steadySpeed);
                } else {
                    log.warn("Maneuver generated by Glidepath did not fit an expected maneuver profile!");
                }
            }
        }

        setAvailability(false);
        return new TrajectoryPlanningResponse();
    }

    /**
     * Repackages all of the MAP & SPAT messages received from the ASD into a single list of intersections
     * that is usable by the Trajectory class.  All of this activity is done in the holder object whose
     * reference is passed in.
     * @param holder - data holder that contains all of the affected data elements
     */
    private void packageAsdMessages(DataElementHolder holder) {
        IntersectionCollection collection = new IntersectionCollection();
    
        //loop through all MAP messages in the holder
        List<IAsdMessage> mapList = ((IAsdListDataElement)holder.get(DataElementKey.MAP_LIST)).value();
        if (mapList != null) {
            for (IAsdMessage msg : mapList) {
                //get its intersections ID
                int id = msg.getIntersectionId();
                //search for this ID in the existing list
                int index = -1;
                if (collection.intersections != null  &&  collection.intersections.size() > 0) {
                    for (int i = 0; i < collection.intersections.size(); ++i) {
                        if (id ==  collection.intersections.get(i).intersectionId) {
                            index = i;
                            break;
                        }
                    }
                }
                //if the ID isn't already represented in the collection then
                if (index < 0) {
                    //add it
                    IntersectionData inter = new IntersectionData();
                    inter.intersectionId = id;
                    inter.map = (MapMessage)msg;
                    if (collection.intersections == null) {
                        collection.intersections = new ArrayList<IntersectionData>();
                    }
                    collection.intersections.add(inter);
                //else (we have info on this intersections already)
                }else {
                    //if this one's version is more recent than the MAP already present, replace it
                    int existingVersion = collection.intersections.get(index).map.getContentVersion();
                    MapMessage mapMsg = (MapMessage)msg;
                    int newVersion = mapMsg.getContentVersion();
                    if (newVersion > existingVersion  || (newVersion == 0  &&  existingVersion != 0)) { //allows for wrap-around
                        log.debug("Ready to replace the existing mapMsg with new version.");
                        collection.intersections.get(index).map = mapMsg;
                    }
                }
            }
        }
        log.debug("packageAsdMessages - map messages complete.");

        //loop through all SPAT messages in the holder
        List<IAsdMessage> spatList = ((IAsdListDataElement)holder.get(DataElementKey.SPAT_LIST)).value();
        if (spatList != null) {
            log.debug("spatList contains " + spatList.size() + " items.");
            for (IAsdMessage msg : spatList) {
                //get its intersections ID
                int id = msg.getIntersectionId();
                //search for this ID in the existing list
                int index = -1;
                if (collection.intersections != null  &&  collection.intersections.size() > 0) {
                    for (int i = 0; i < collection.intersections.size(); ++i) {
                        if (id == collection.intersections.get(i).intersectionId) {
                            index = i;
                            break;
                        }
                    }
                }
                SpatMessage spatMsg = (SpatMessage)msg;

                //if the ID isn't already represented in the collection then
                if (index < 0) {
                    //add it
                    IntersectionData inter = new IntersectionData();
                    inter.intersectionId = id;
                    inter.spat = spatMsg;
                    if (collection.intersections == null) {
                        collection.intersections = new ArrayList<IntersectionData>();
                    }
                    collection.intersections.add(inter);
                }else {
                    //if this one's version number is larger (with wrapping), replace it
                    int existingVersion;
                    SpatMessage prevSpat = (SpatMessage)collection.intersections.get(index).spat;
                    if (prevSpat == null) {
                        existingVersion = -1;
                    }else {
                        existingVersion = prevSpat.getContentVersion();
                    }
                    int newVersion = spatMsg.getContentVersion();
                    if (newVersion > existingVersion  ||  (newVersion == 0  &&  existingVersion != 0)) {
                        collection.intersections.get(index).spat = spatMsg;
                    }
                }
            }
        }
        log.debug("packageAsdMessages - spat messages complete.");

        //remove the MAP & SPAT lists from the data holder
        holder.remove(DataElementKey.MAP_LIST);
        holder.remove(DataElementKey.SPAT_LIST);

        //add the new intersections collection to the holder
        holder.put(DataElementKey.INTERSECTION_COLLECTION, new IntersectionCollectionDataElement(collection));
    }

}
