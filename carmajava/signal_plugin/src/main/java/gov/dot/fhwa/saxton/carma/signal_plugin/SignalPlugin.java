package gov.dot.fhwa.saxton.carma.signal_plugin;

import java.util.ArrayList;
import java.util.List;

import org.ros.message.Time;
import org.ros.rosjava_geometry.Transform;

import cav_msgs.MapData;
import cav_msgs.SPAT;
import geometry_msgs.Twist;
import geometry_msgs.TwistStamped;
import gov.dot.fhwa.saxton.carma.geometry.cartesian.Point3D;
import gov.dot.fhwa.saxton.carma.guidance.arbitrator.TrajectoryPlanningResponse;
import gov.dot.fhwa.saxton.carma.guidance.plugins.AbstractPlugin;
import gov.dot.fhwa.saxton.carma.guidance.plugins.IStrategicPlugin;
import gov.dot.fhwa.saxton.carma.guidance.plugins.PluginServiceLocator;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.ISubscriber;
import gov.dot.fhwa.saxton.carma.guidance.trajectory.Trajectory;
import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.DataElementHolder;
import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.DataElementKey;
import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.IntersectionCollectionDataElement;
import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.IAsdListDataElement;;
import gov.dot.fhwa.saxton.carma.signal_plugin.asd.IAsdMessage;
import gov.dot.fhwa.saxton.carma.signal_plugin.asd.IntersectionCollection;
import gov.dot.fhwa.saxton.carma.signal_plugin.asd.IntersectionData;
import gov.dot.fhwa.saxton.carma.signal_plugin.asd.map.MapMessage;
import gov.dot.fhwa.saxton.carma.signal_plugin.asd.spat.SpatMessage;
import gov.dot.fhwa.saxton.carma.signal_plugin.ead.EadAStar;
import gov.dot.fhwa.saxton.carma.signal_plugin.ead.IEad;
import sensor_msgs.NavSatFix;;

/**
 * Top level class in the Signal Plugin that does trajectory planning through signalized intersections.
 * This is a port of the functionality developed under the STOL I contract TO 17 and STOL II contract
 * TO 13, Glidepath project.
 */
public class SignalPlugin extends AbstractPlugin implements IStrategicPlugin {

    private IEad eadAlgorithm;
    private ISubscriber<MapData> mapSub;
    private ISubscriber<SPAT> spatSub;
    private ISubscriber<NavSatFix> gpsSub;
    private ISubscriber<TwistStamped> velSub;

    public SignalPlugin(PluginServiceLocator psl) {
        super(psl);
        version.setName("Signal Plugin");
        version.setMajorRevision(1);
        version.setIntermediateRevision(0);
        version.setMinorRevision(0);
    }

    //TODO: this is a skeleton!  Unless otherwise noted, all comments below are PDL that need to be expanded to working code


    @Override
    public void onInitialize() {
        //load params

        log.info("STARTUP", "SignalPlugin has been initialized.");
        //log the key params here
        eadAlgorithm = new EadAStar();

        mapSub = pluginServiceLocator.getPubSubService().getSubscriberForTopic("map", MapData._TYPE);

        mapSub.registerOnMessageCallback((map) -> {

        });

        spatSub = pluginServiceLocator.getPubSubService().getSubscriberForTopic("spat", SPAT._TYPE);
        gpsSub = pluginServiceLocator.getPubSubService().getSubscriberForTopic("nav_sat_fix", NavSatFix._TYPE);
        velSub = pluginServiceLocator.getPubSubService().getSubscriberForTopic("velocity", TwistStamped._TYPE);
    }


    @Override
    public void onResume() {

        log.info("SignalPlugin has resumed.");
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


    @Override
    public TrajectoryPlanningResponse planTrajectory(Trajectory traj, double expectedStartSpeed) {





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

    /**
     * TODO
     * Gets the transform of between the requested frames
     * The transform describes the location of the child frame in the parent frame
     * 
     * @param parentFrame Frame Id of parent frame
     * @param childFrame Frame Id of child frame
     * @param stamp The time which this transform should correspond to
     */
    private Transform getTransform(String parentFrame, String childFrame, Time stamp) {
        return null;
    }

}
