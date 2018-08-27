package gov.dot.fhwa.saxton.carma.signal_plugin;

import java.util.Comparator;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Map.Entry;
import java.util.concurrent.atomic.AtomicReference;

import org.ros.message.Time;

import cav_msgs.RoadwayObstacle;
import gov.dot.fhwa.saxton.carma.guidance.ArbitratorService;
import gov.dot.fhwa.saxton.carma.guidance.conflictdetector.IConflictDetector;
import gov.dot.fhwa.saxton.carma.guidance.plugins.PluginServiceLocator;
import gov.dot.fhwa.saxton.carma.guidance.util.RouteService;
import gov.dot.fhwa.saxton.carma.guidance.util.trajectoryconverter.RoutePointStamped;
import gov.dot.fhwa.saxton.carma.rosutils.SaxtonLogger;
import gov.dot.fhwa.saxton.carma.signal_plugin.ead.IMotionInterpolator;
import gov.dot.fhwa.saxton.carma.signal_plugin.ead.IMotionPredictor;
import gov.dot.fhwa.saxton.carma.signal_plugin.ead.INodeCollisionChecker;
import gov.dot.fhwa.saxton.carma.signal_plugin.ead.trajectorytree.Node;

/**
 *
 */
public class ObjectCollisionChecker implements INodeCollisionChecker {

  IConflictDetector conflictDetector;

  // Tracked objects
  Map<Integer, PriorityQueue<RoadwayObstacle>> trackedLaneObjectsHistory = new HashMap<>();  
  Map<Integer, List<RoutePointStamped>> trackedLaneObjectsPredictions = new HashMap<>();

  AtomicReference<List<RoutePointStamped>> interpolatedHostPlan;

  private SaxtonLogger log;
  private RouteService routeService;
  private ArbitratorService arbitratorService;

  private final IMotionPredictor motionPredictor;
  private final IMotionInterpolator motionInterpolator;
  private final long maxHistoricalDataAge; // ms
  private final double distanceStep; // m
  private final double timeDuration; // s
  private final double timeRange; // s
  private final double distanceBuffer; // m
  private final double crosstrackBuffer; // m
  private final long NCVReplanPeriod; // ms
  private final double MS_PER_S = 1000.0;
  private Long ncvDetectionTime = null;

  public ObjectCollisionChecker(PluginServiceLocator psl, SaxtonLogger log,
    IMotionPredictorModelFactory modelFactory, IMotionInterpolator motionInterpolator) {
    this.routeService = psl.getRouteService();
    this.arbitratorService = psl.getArbitratorService();

    String predictionModel = psl.getParameterSource().getString("ead.NCVHandling.objectMotionPredictorModel");
    this.motionPredictor = modelFactory.getMotionPredictor(predictionModel);

    this.maxHistoricalDataAge = psl.getParameterSource().getInteger("ead.NCVHandling.collision.maxObjectHistoricalDataAge");
    this.distanceStep = psl.getParameterSource().getDouble("ead.NCVHandling.collision.distanceStep");
    this.timeDuration = psl.getParameterSource().getDouble("ead.NCVHandling.collision.timeDuration");
    this.NCVReplanPeriod = (long) ((timeDuration / 2.0) * MS_PER_S); // Always replan after half of the ncv prediction has elapsed
    this.timeRange = psl.getParameterSource().getDouble("ead.NCVHandling.collision.timeRange");
    this.distanceBuffer = psl.getParameterSource().getDouble("ead.NCVHandling.collision.distanceBuffer");
    this.crosstrackBuffer = psl.getParameterSource().getDouble("ead.NCVHandling.collision.crosstrackBuffer");

    this.motionInterpolator = motionInterpolator;
    
  }

  public void updateObjects(List<RoadwayObstacle> obstacles) {

    // Iterate over detected objects and keep only those in front of us in the same lane. 
    int currentLane = routeService.getCurrentRouteSegment().determinePrimaryLane(routeService.getCurrentCrosstrackDistance());
    int inLaneObjectCount = 0;

    for (RoadwayObstacle obs : obstacles) {

      double frontObjectDistToCenters =  obs.getDownTrack() - routeService.getCurrentDowntrackDistance();
      boolean inLane = obs.getPrimaryLane() == currentLane;

      // If the object is in the same lane and in front of the host vehicle
      // Add it to the set of tracked object histories
      // TODO this currently does not handle if the lane index changes between the host and detected object
      if (inLane && frontObjectDistToCenters > -0.0) {
        
        inLaneObjectCount++;
        if (!trackedLaneObjectsHistory.containsKey(obs.getObject().getId())) {
          // Sort object history as time sorted priority queue
          PriorityQueue<RoadwayObstacle> objectHistory = new PriorityQueue<RoadwayObstacle>(new Comparator<RoadwayObstacle> (){
            public int compare(RoadwayObstacle r1, RoadwayObstacle r2) {
              // Sort objects so oldest objects are at the front of the queue
              return r1.getObject().getHeader().getStamp().compareTo(r2.getObject().getHeader().getStamp());
            }
          });
          trackedLaneObjectsHistory.put(obs.getObject().getId(), objectHistory);
        }

        PriorityQueue<RoadwayObstacle> objHistory = trackedLaneObjectsHistory.get(obs.getObject().getId());
        
        // Add new historical data and remove old data to maintain queue size if needed
        objHistory.add(obs);
      }
    }

    // Predict and cache the motion of each object
    // Also remove expired data and identify expired objects
    final Time minObjStamp = Time.fromMillis(System.currentTimeMillis() - maxHistoricalDataAge);
    List<Integer> expiredObjIds = new LinkedList<>(); // List of objects which are expired and marked for removal

    for (Entry<Integer, PriorityQueue<RoadwayObstacle>> e: trackedLaneObjectsHistory.entrySet()) {

      removeExpiredData(e.getValue(), minObjStamp); // Remove expired history data
      
      if (e.getValue().isEmpty()) { // Check if an object is totally expired
        expiredObjIds.add(e.getKey());
        continue; // No point in computing prediction if the object is expired anyway
      }

      trackedLaneObjectsPredictions.put(
        e.getKey(), 
        motionPredictor.predictMotion(e.getKey().toString(), e.getValue(), distanceStep, timeDuration)
      );
    }

    // Remove expired objects
    for (Integer objId: expiredObjIds) {
      trackedLaneObjectsHistory.remove(objId);
      trackedLaneObjectsPredictions.remove(objId);
    }

    // Check for collisions using new object data
    boolean collisionDetected = checkCollision(interpolatedHostPlan.get());

    if (collisionDetected) { // Any time there is a detected collision force a replan
      ncvDetectionTime = System.currentTimeMillis();
      arbitratorService.requestNewPlan(); // Request a replan from the arbitrator

    } else if (ncvDetectionTime != null && System.currentTimeMillis() - ncvDetectionTime > NCVReplanPeriod){ // If there was previously a replan to avoid a ncv and enough time has passed replan
      ncvDetectionTime = System.currentTimeMillis();
      arbitratorService.requestNewPlan(); // Request a replan from the arbitrator

    } else if (inLaneObjectCount == 0) { // If no in lane objects were detected the ncv is no longer an issue. The assumption being made here is that sensor fusion has already filtered our incoming object data.
      
      ncvDetectionTime = null;
    } else {
      // There is no need to take action when the plan is executing well without collisions
    }
  }

  private void removeExpiredData(PriorityQueue<RoadwayObstacle> objHistory, Time minObjStamp) {

    Time oldestObjStamp = objHistory.peek().getObject().getHeader().getStamp();
    while (oldestObjStamp != null && oldestObjStamp.compareTo(minObjStamp) < 0) {
      objHistory.poll();
      oldestObjStamp = objHistory.peek().getObject().getHeader().getStamp();
    }
  }

  public void setHostPlan(List<Node> hostPlan) {
    interpolatedHostPlan.set(motionInterpolator.interpolateMotion(hostPlan, distanceStep));
  }

  private boolean checkCollision(List<RoutePointStamped> routePlan) {
    // Check the proposed trajectory against all tracked objects for collisions
    for (Entry<Integer, List<RoutePointStamped>> objPrediction: trackedLaneObjectsPredictions.entrySet()) {
      // Check for conflicts against each object and return true if any conflict is found
      if (!conflictDetector.getConflicts(routePlan, objPrediction.getValue()).isEmpty()) { // TODO we need to pass in the appropriate object bounds
        return true;
      }
    }

    return false;
  }

  @Override
  public boolean hasCollision(List<Node> trajectory) {
    
    // Convert the proposed trajectory to route points
    List<RoutePointStamped> routePlan = motionInterpolator.interpolateMotion(trajectory, distanceStep);

    return checkCollision(routePlan); // Check for collisions with tracked objects

  }
}
