package gov.dot.fhwa.saxton.carma.signal_plugin;

import java.util.ArrayList;
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
import gov.dot.fhwa.saxton.carma.guidance.conflictdetector.ConflictSpace;
import gov.dot.fhwa.saxton.carma.guidance.conflictdetector.IConflictDetector;
import gov.dot.fhwa.saxton.carma.guidance.plugins.PluginServiceLocator;
import gov.dot.fhwa.saxton.carma.guidance.util.ILogger;
import gov.dot.fhwa.saxton.carma.guidance.util.ITimeProvider;
import gov.dot.fhwa.saxton.carma.guidance.util.LoggerManager;
import gov.dot.fhwa.saxton.carma.guidance.util.RouteService;
import gov.dot.fhwa.saxton.carma.guidance.util.trajectoryconverter.RoutePointStamped;
import gov.dot.fhwa.saxton.carma.rosutils.SaxtonLogger;
import gov.dot.fhwa.saxton.carma.signal_plugin.ead.IMotionInterpolator;
import gov.dot.fhwa.saxton.carma.signal_plugin.ead.IMotionPredictor;
import gov.dot.fhwa.saxton.carma.signal_plugin.ead.INodeCollisionChecker;
import gov.dot.fhwa.saxton.carma.signal_plugin.ead.trajectorytree.Node;

/**
 * The ObjectCollisionChecker will provide collision checking functionality for the Traffic Signal Plugin
 * The historical descriptions of detected in lane objects are cached and used to predict future object motion
 * The object motion predictions are used for collision checking to prevent the plugin from planning a path through a detected object
 * Additionally, when an upcoming collision is detected based on new data, a total replan will be requested
 * After a replan occurs additional replans will be requested at time increments equal to half the prediction period or when a new collision is identified
 * This system is capable of handling multiple in lane objects, but makes the assumption that lane id's will not change between the host vehicle and the detected objects.
 */
public class ObjectCollisionChecker implements INodeCollisionChecker {

  private final IConflictDetector conflictDetector;

  // Tracked objects
  Map<Integer, PriorityQueue<RoadwayObstacle>> trackedLaneObjectsHistory = new HashMap<>();  
  Map<Integer, List<RoutePointStamped>> trackedLaneObjectsPredictions = new HashMap<>();

  private final AtomicReference<List<RoutePointStamped>> interpolatedHostPlan = new AtomicReference<>(new LinkedList<>()); // Current Host Plan

  private final ILogger log;
  private final RouteService routeService;
  private final ArbitratorService arbitratorService;
  private final ITimeProvider timeProvider;

  private final IMotionPredictor motionPredictor;
  private final IMotionInterpolator motionInterpolator;
  private final long maxHistoricalDataAge; // ms
  private final double distanceStep; // m
  private final double timeDuration; // s
  private final double downtrackBuffer; // m
  private final double crosstrackBuffer; // m

  private final double timeMargin; //s
  private final double downtrackMargin; // m
  private final double crosstrackMargin; // m

  private final double longitudinalBias;
  private final double lateralBias;
  private final double temporalBias;


  private final long NCVReplanPeriod; // ms
  private final double MS_PER_S = 1000.0; // ms
  private Long ncvDetectionTime = null; // ms

  /**
   * Constructor
   * 
   * @param psl The plugin service locator used to load parameters and route details
   * @param modelFactory The factory used for getting an IMotionPredictor to predict object trajectories
   * @param motionInterpolator The host vehicle motion interpolator which will interpolate vehicle plans as needed
   */
  public ObjectCollisionChecker(PluginServiceLocator psl,
    IMotionPredictorModelFactory modelFactory, IMotionInterpolator motionInterpolator) {
    this.log = LoggerManager.getLogger();
    this.routeService = psl.getRouteService();
    this.arbitratorService = psl.getArbitratorService();
    this.timeProvider = psl.getTimeProvider();
    this.conflictDetector = psl.getConflictDetector();

    String predictionModel = psl.getParameterSource().getString("~ead/NCVHandling/objectMotionPredictorModel");
    this.motionPredictor = modelFactory.getMotionPredictor(predictionModel);

    this.maxHistoricalDataAge = psl.getParameterSource().getInteger("~ead/NCVHandling/collision/maxObjectHistoricalDataAge");
    this.distanceStep = psl.getParameterSource().getDouble("~ead/NCVHandling/collision/distanceStep");
    this.timeDuration = psl.getParameterSource().getDouble("~ead/NCVHandling/collision/timeDuration");
    this.NCVReplanPeriod = (long) ((timeDuration / 2.0) * MS_PER_S); // Always replan after half of the ncv prediction has elapsed

    this.downtrackBuffer = psl.getParameterSource().getDouble("~ead/NCVHandling/collision/downtrackBuffer");
    this.crosstrackBuffer = psl.getParameterSource().getDouble("~ead/NCVHandling/collision/crosstrackBuffer");
    
    // The downtrack and crosstrack margins are half the vehicle dimension plus the amount of buffer
    this.downtrackMargin = (psl.getParameterSource().getDouble("vehicle_length") / 2.0) + this.downtrackBuffer;
    this.crosstrackMargin = (psl.getParameterSource().getDouble("vehicle_width") / 2.0) + this.crosstrackBuffer;
    this.timeMargin = psl.getParameterSource().getDouble("~ead/NCVHandling/collision/timeMargin");

    this.longitudinalBias = psl.getParameterSource().getDouble("~ead/NCVHandling/collision/longitudinalBias");
    this.lateralBias = psl.getParameterSource().getDouble("~ead/NCVHandling/collision/lateralBias");
    this.temporalBias = psl.getParameterSource().getDouble("~ead/NCVHandling/collision/temporalBias");

    this.motionInterpolator = motionInterpolator;
    
  }

  /**
   * Function to be called when new objects are detected by host vehicle sensors
   * This will update object histories and predictions
   * If a collision is detected based on new predictions then a replan will be requested
   * 
   * Note: This function assumes this data will be provided at a fixed rate which is below half the prediction period 
   * 
   * @param obstacles The list of detected objects in route space around the vehicle
   */
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
    final Time minObjStamp = Time.fromMillis(timeProvider.getCurrentTimeMillis() - maxHistoricalDataAge);
    List<Integer> expiredObjIds = new LinkedList<>(); // List of objects which are expired and marked for removal

    for (Entry<Integer, PriorityQueue<RoadwayObstacle>> e: trackedLaneObjectsHistory.entrySet()) {

      removeExpiredData(e.getValue(), minObjStamp); // Remove expired history data
      
      if (e.getValue().isEmpty()) { // Check if an object is totally expired
        expiredObjIds.add(e.getKey());
        continue; // No point in computing prediction if the object is expired anyway
      }

      trackedLaneObjectsPredictions.put(
        e.getKey(), 
        motionPredictor.predictMotion(e.getKey().toString(), new ArrayList<>(e.getValue()), distanceStep, timeDuration)
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
      ncvDetectionTime = timeProvider.getCurrentTimeMillis();
      arbitratorService.requestNewPlan(); // Request a replan from the arbitrator

    } else if (ncvDetectionTime != null && timeProvider.getCurrentTimeMillis() - ncvDetectionTime > NCVReplanPeriod){ // If there was previously a replan to avoid a ncv and enough time has passed replan
      ncvDetectionTime = timeProvider.getCurrentTimeMillis();
      arbitratorService.requestNewPlan(); // Request a replan from the arbitrator

    } else if (inLaneObjectCount == 0) { // If no in lane objects were detected the ncv is no longer an issue. The assumption being made here is that sensor fusion has already filtered our incoming object data.
      
      ncvDetectionTime = null;
    } else {
      // There is no need to take action when the plan is executing well without collisions
    }
  }

  /**
   * Helper function to remove expired data elements from an objects historical data
   * 
   * @param objHistory Objects historical data
   * @param minObjStamp The earliest time which a data element can have without being removed
   */
  private void removeExpiredData(PriorityQueue<RoadwayObstacle> objHistory, Time minObjStamp) {

    RoadwayObstacle obs = objHistory.peek();
    while (obs != null && obs.getObject().getHeader().getStamp().compareTo(minObjStamp) < 0) {
      objHistory.poll();
      obs = objHistory.peek();
    }
  }

  /**
   * Sets the current host vehicle plan which will be used for collision checks
   * The provided host plan will be interpolated using the provided IMotionInterpolator
   * 
   * @param hostPlan The host plan to set as the current plan
   * @param startTime The time which planning is considered to have begun at. This is used for converting nodes to route locations
	 * @param startDowntrack The downtrack distance where planning is considered to have begun at. This is used for converting nodes to route locations
	 * 
   */
  public void setHostPlan(List<Node> hostPlan, double startTime, double startDowntrack) {
    interpolatedHostPlan.set(motionInterpolator.interpolateMotion(hostPlan, distanceStep,
      startTime, startDowntrack
    ));
  }

  /**
   * Helper function to check collisions between predicted object trajectories and the provided plan
   * 
   * @param routePlan The plan to check for collisions with
   * 
   * @return True if a collision was found. False otherwise
   */
  private boolean checkCollision(List<RoutePointStamped> routePlan) {
    // Check the proposed trajectory against all tracked objects for collisions
    for (Entry<Integer, List<RoutePointStamped>> objPrediction: trackedLaneObjectsPredictions.entrySet()) {
      List<RoutePointStamped> objPlan = objPrediction.getValue();
      double dynamicTimeMargin = timeMargin;
      // Compute an estimated time margin to ensure overlap of collision bounds
      if (objPrediction.getValue().size() > 1) {
        // TODO this assumes linear regression used for motion prediction resulting in constant slope
        // The time margin should be half delta t plus a small bit of overlap
        dynamicTimeMargin = ((objPlan.get(1).getStamp() - objPlan.get(0).getStamp()) / 2.0) + 0.0001;
      }
      // Check for conflicts against each object and return true if any conflict is found
      List<ConflictSpace> conflictSpaces = conflictDetector.getConflicts(
        routePlan, objPlan,
        downtrackMargin, crosstrackMargin, dynamicTimeMargin,
        longitudinalBias, lateralBias, temporalBias
      );
      
      if (!conflictSpaces.isEmpty()) {
        return true;
      }
    }

    return false;
  }

  @Override
  public boolean hasCollision(List<Node> trajectory, double timeOffset, double distanceOffset) {
    
    // Convert the proposed trajectory to route points
    List<RoutePointStamped> routePlan = motionInterpolator.interpolateMotion(trajectory, distanceStep,
      timeOffset, distanceOffset
    );

    return checkCollision(routePlan); // Check for collisions with tracked objects

  }
}
