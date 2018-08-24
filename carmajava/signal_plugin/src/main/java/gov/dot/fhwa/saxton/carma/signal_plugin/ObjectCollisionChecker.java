package gov.dot.fhwa.saxton.carma.signalplugin;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Map.Entry;

import org.apache.commons.logging.Log;
import org.ros.rosjava_geometry.Transform;

import cav_msgs.MapData;
import cav_msgs.RoadwayObstacle;
import cav_msgs.SPAT;
import cav_srvs.GetTransformRequest;
import geometry_msgs.Twist;
import geometry_msgs.TwistStamped;
import gov.dot.fhwa.saxton.carma.geometry.cartesian.Point3D;
import gov.dot.fhwa.saxton.carma.guidance.arbitrator.TrajectoryPlanningResponse;
import gov.dot.fhwa.saxton.carma.guidance.plugins.AbstractPlugin;
import gov.dot.fhwa.saxton.carma.guidance.plugins.IStrategicPlugin;
import gov.dot.fhwa.saxton.carma.guidance.plugins.PluginServiceLocator;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.ISubscriber;
import gov.dot.fhwa.saxton.carma.guidance.trajectory.Trajectory;
import gov.dot.fhwa.saxton.carma.guidance.util.RouteService;
import gov.dot.fhwa.saxton.carma.rosutils.SaxtonLogger;
import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.DataElementHolder;
import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.DataElementKey;
import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.IntersectionCollectionDataElement;
import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.IAsdListDataElement;
import gov.dot.fhwa.saxton.carma.signal_plugin.asd.IAsdMessage;
import gov.dot.fhwa.saxton.carma.signal_plugin.asd.IntersectionCollection;
import gov.dot.fhwa.saxton.carma.signal_plugin.asd.IntersectionData;
import gov.dot.fhwa.saxton.carma.signal_plugin.asd.map.MapMessage;
import gov.dot.fhwa.saxton.carma.signal_plugin.asd.spat.SpatMessage;
import gov.dot.fhwa.saxton.carma.signal_plugin.ead.EadAStar;
import gov.dot.fhwa.saxton.carma.signal_plugin.ead.IEad;
import gov.dot.fhwa.saxton.carma.signal_plugin.ead.IMotionInterpolator;
import gov.dot.fhwa.saxton.carma.signal_plugin.ead.IMotionPredictor;
import gov.dot.fhwa.saxton.carma.signal_plugin.ead.INodeCollisionChecker;
import gov.dot.fhwa.saxton.carma.signal_plugin.ead.trajectorytree.Node;
import sensor_msgs.NavSatFix;;

/**
 *
 */
public class ObjectCollisionChecker implements INodeCollisionChecker {

  private IEad eadAlgorithm;
  private ISubscriber<MapData> mapSub;
  private ISubscriber<SPAT> spatSub;
  private ISubscriber<NavSatFix> gpsSub;
  private ISubscriber<TwistStamped> velSub;

  private SaxtonLogger log;
  private Transform hostVehicleToVehicleFront;
  private RouteService routeService;

  public ObjectCollisionChecker(PluginServiceLocator psl, SaxtonLogger log, String hostVehicleFrame, String vehicleFrontFrame) {
    psl.getRouteService();
  }

  public void updateObjects(List<RoadwayObstacle> obstacles) {
    if (hostVehicleToVehicleFront == null) {
      // Transform is static so we only need it once
      hostVehicleToVehicleFront = plugin.getTransform(hostVehicleFrame, vehicleFrontFrame, Time.fromMillis(0));
      if (hostVehicleToVehicleFront == null)
          return;
    }

    Map<Integer, PriorityQueue<RoadwayObstacle>> trackedLaneObjects = new HashMap<>(); 

    int currentLane = routeService.getCurrentRouteSegment().determinePrimaryLane(routeService.getCurrentCrosstrackDistance());
    for (RoadwayObstacle obs : obstacles) {

      double frontObjectDistToCenters =  obs.getDownTrack() - routeService.getCurrentDowntrackDistance();
      boolean inLane = obs.getPrimaryLane() == currentLane;

      // If the object is in the same lane and in front of the host vehicle
      // Add it to the set of tracked object histories
      if (inLane && frontObjectDistToCenters > -0.0) {

        if (!trackedLaneObjects.containsKey(obs.getObject().getId())) {
          // Sort object history as time sorted priority queue
          PriorityQueue<RoadwayObstacle> objectHistory = new PriorityQueue<RoadwayObstacle>(new Comparator<RoadwayObstacle> (){
            public int compare(RoadwayObstacle r1, RoadwayObstacle r2) {
              // Assume all objects are in front of host vehicle and in same lane TODO comment
              return r1.getObject().getHeader().getStamp().compareTo(r2.getObject().getHeader().getStamp());
            }
          });
          trackedLaneObjects.put(obs.getObject().getId(), objectHistory);
        }

        PriorityQueue<RoadwayObstacle> objHistory = trackedLaneObjects.get(obs.getObject().getId());
        
        // Add new historical data and remove old data to maintain queue size if needed
        objHistory.add(obs);

        if (objHistory.size() > maxObjectHistory) {
          objHistory.poll();
        }
      }
    }

    // TODO we need to remove expired objects from map

    double distanceStep, timeDuration; //TODO

    // TODO we have predicted the motion of each in lane object but not we need to store that for checks against
    IMotionPredictor motionPredictor; // TODO
    for (Entry<Integer, PriorityQueue<RoadwayObstacle>> e: trackedLaneObjects.entrySet()) {
      motionPredictor.predictMotion(e.getKey().toString(), e.getValue(), distanceStep, timeDuration);
    }
    
  }

  @Override
  public boolean hasCollision(List<Node> trajectory) {
    return false;
  }
}
