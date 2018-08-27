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
import gov.dot.fhwa.saxton.carma.guidance.conflictdetector.IConflictDetector;
import gov.dot.fhwa.saxton.carma.guidance.plugins.AbstractPlugin;
import gov.dot.fhwa.saxton.carma.guidance.plugins.IStrategicPlugin;
import gov.dot.fhwa.saxton.carma.guidance.plugins.PluginServiceLocator;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.ISubscriber;
import gov.dot.fhwa.saxton.carma.guidance.trajectory.Trajectory;
import gov.dot.fhwa.saxton.carma.guidance.util.RouteService;
import gov.dot.fhwa.saxton.carma.guidance.util.trajectoryconverter.RoutePointStamped;
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

  IConflictDetector conflictDetector;

  // Tracked objects
  Map<Integer, PriorityQueue<RoadwayObstacle>> trackedLaneObjectsHistory = new HashMap<>();  
  Map<Integer, List<RoutePointStamped>> trackedLaneObjectsPredictions = new HashMap<>();

  private SaxtonLogger log;
  private RouteService routeService;

  private final int objectHistoricalSampleCount;
  private final double distanceStep;
  private final double timeDuration;
  private final double timeRange;
  private final double distanceBuffer;
  private final double crosstrackBuffer;

  public ObjectCollisionChecker(PluginServiceLocator psl, SaxtonLogger log) {
    routeService = psl.getRouteService();
    psl.getParameterSource().getString("ead.NCVHandling.objectMotionPredictorModel");
    psl.getParameterSource().getInteger("ead.NCVHandling.objectHistoricalSampleCount");
    psl.getParameterSource().getDouble("ead.NCVHandling.collision.distanceStep");
    psl.getParameterSource().getDouble("ead.NCVHandling.collision.timeDuration");
    psl.getParameterSource().getDouble("ead.NCVHandling.collision.timeRange");
    psl.getParameterSource().getDouble("ead.NCVHandling.collision.distanceBuffer");
    psl.getParameterSource().getDouble("ead.NCVHandling.collision.crosstrackBuffer");
  }

  public void updateObjects(List<RoadwayObstacle> obstacles) {

    // Iterate over detected objects and keep only those in front of us in the same lane. 
    int currentLane = routeService.getCurrentRouteSegment().determinePrimaryLane(routeService.getCurrentCrosstrackDistance());
    for (RoadwayObstacle obs : obstacles) {

      double frontObjectDistToCenters =  obs.getDownTrack() - routeService.getCurrentDowntrackDistance();
      boolean inLane = obs.getPrimaryLane() == currentLane;

      // If the object is in the same lane and in front of the host vehicle
      // Add it to the set of tracked object histories
      if (inLane && frontObjectDistToCenters > -0.0) {

        if (!trackedLaneObjectsHistory.containsKey(obs.getObject().getId())) {
          // Sort object history as time sorted priority queue
          PriorityQueue<RoadwayObstacle> objectHistory = new PriorityQueue<RoadwayObstacle>(new Comparator<RoadwayObstacle> (){
            public int compare(RoadwayObstacle r1, RoadwayObstacle r2) {
              // Assume all objects are in front of host vehicle and in same lane TODO comment
              return r1.getObject().getHeader().getStamp().compareTo(r2.getObject().getHeader().getStamp());
            }
          });
          trackedLaneObjectsHistory.put(obs.getObject().getId(), objectHistory);
        }

        PriorityQueue<RoadwayObstacle> objHistory = trackedLaneObjectsHistory.get(obs.getObject().getId());
        
        // Add new historical data and remove old data to maintain queue size if needed
        objHistory.add(obs);

        if (objHistory.size() > maxObjectHistory) {
          objHistory.poll();
        }
      }
    }

    // TODO we need to remove expired objects from map

    double distanceStep, timeDuration; //TODO

    // Predict and cache the motion of each object
    IMotionPredictor motionPredictor; // TODO
    for (Entry<Integer, PriorityQueue<RoadwayObstacle>> e: trackedLaneObjectsHistory.entrySet()) {
      trackedLaneObjectsPredictions.put(
        e.getKey(), 
        motionPredictor.predictMotion(e.getKey().toString(), e.getValue(), distanceStep, timeDuration)
      );
    }
  }

  @Override
  public boolean hasCollision(List<Node> trajectory) {
    
    List<RoutePointStamped> hostPlan; // TODO
    for (Entry<Integer, List<RoutePointStamped>> objPrediction: trackedLaneObjectsPredictions.entrySet()) {
      // Check for conflicts against each object and return true if any conflict is found
      if (!conflictDetector.getConflicts(hostPlan, objPrediction.getValue()).isEmpty()) {
        return true;
      }
    }

    return false;
  }
}
