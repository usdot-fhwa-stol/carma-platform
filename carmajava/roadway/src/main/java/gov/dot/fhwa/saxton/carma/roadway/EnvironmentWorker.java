/*
 * Copyright (C) 2017 LEIDOS.
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

package gov.dot.fhwa.saxton.carma.roadway;

import cav_msgs.RoadwayObstacle;
import cav_msgs.RouteState;
import cav_msgs.SystemAlert;
import geometry_msgs.TransformStamped;
import gov.dot.fhwa.saxton.carma.geometry.GeodesicCartesianConverter;
import gov.dot.fhwa.saxton.carma.geometry.cartesian.Point3D;
import gov.dot.fhwa.saxton.carma.geometry.cartesian.QuaternionUtils;
import gov.dot.fhwa.saxton.carma.geometry.cartesian.Vector3D;
import gov.dot.fhwa.saxton.carma.geometry.geodesic.Location;
import gov.dot.fhwa.saxton.carma.rosutils.SaxtonLogger;
import gov.dot.fhwa.saxton.carma.route.Route;
import gov.dot.fhwa.saxton.carma.route.RouteSegment;
import gov.dot.fhwa.saxton.carma.route.RouteWaypoint;
import gov.dot.fhwa.saxton.carma.route.WorkerState;

import org.apache.commons.logging.Log;
import org.ros.message.Duration;
import org.ros.message.MessageFactory;
import org.ros.message.Time;
import org.ros.node.NodeConfiguration;
import org.ros.rosjava_geometry.Quaternion;
import org.ros.rosjava_geometry.Transform;
import org.ros.rosjava_geometry.Vector3;
import std_msgs.Header;
import tf2_msgs.TFMessage;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

/**
 * The EnvironmentWorker is responsible for implementing all non pub-sub logic of the EnvironmentManager node
 * Primary responsibility is updating of coordinate transforms and providing a representation of local geometry
 */
public class EnvironmentWorker {
  // Messaging and logging
  protected SaxtonLogger log;
  protected IRoadwayManager roadwayMgr;
  protected final MessageFactory messageFactory = NodeConfiguration.newPrivate().getTopicMessageFactory();

  // The heading of the vehicle in degrees east of north in an NED frame.
  // Frame ids
  protected final String earthFrame;
  protected final String mapFrame;
  protected final String odomFrame;
  protected final String baseLinkFrame;
  protected final String globalPositionSensorFrame;
  protected final String localPositionSensorFrame;

  //Route
  protected Route activeRoute;
  protected RouteSegment currentSegment;
  protected RouteState routeState;
  protected double distBackward;
  protected double distForward;

  /**
   * Constructor
   *
   * @param roadwayMgr EnvironmentWorker used to publish data and get stored transforms
   * @param log    Logging object
   * @param earthFrame The frame id used to identify the ECEF frame
   * @param mapFrame The frame id used to identify the global map frame
   * @param odomFrame The frame id used to identify the local planning frame
   * @param baseLinkFrame The frame id used to identify the host vehicle frame
   * @param globalPositionSensorFrame The frame id used to identify the frame of a global position sensor
   * @param localPositionSensorFrame The frame id used to identify a local odometry position sensor frame
   * @param distBackward The distance in m uptrack of the host vehicles segment which will be included
   * @param distForward The distance in m downtrack of the host vehicles segment which will be included
   */
  public EnvironmentWorker(IRoadwayManager roadwayMgr, Log log, String earthFrame, String mapFrame,
    String odomFrame, String baseLinkFrame, String globalPositionSensorFrame, String localPositionSensorFrame,
    double distBackward, double distForward) {
    this.log = new SaxtonLogger(this.getClass().getSimpleName(), log);
    this.roadwayMgr = roadwayMgr;
    this.earthFrame = earthFrame;
    this.mapFrame = mapFrame;
    this.odomFrame = odomFrame;
    this.baseLinkFrame = baseLinkFrame;
    this.globalPositionSensorFrame = globalPositionSensorFrame;
    this.localPositionSensorFrame = localPositionSensorFrame;
    this.distBackward = distBackward;
    this.distForward = distForward;
  }

 /**
   * Route message handle
   *
   * @param route The route message
   */
  public void handleRouteMsg(cav_msgs.Route route) {
    activeRoute = Route.fromMessage(route);
  }

  /**
   * RouteState message handle
   *
   * @param route The route state message
   */
  public void handleRouteStateMsg(cav_msgs.RouteState routeState) {
    this.routeState = routeState;
    this.currentSegment = RouteSegment.fromMessage(routeState.getCurrentSegment());
  }

  /**
   * External object message handler
   *
   * @param externalObjects External object list. Should be relative to odom frame
   */
  public void handleExternalObjectsMsg(cav_msgs.ExternalObjectList externalObjects) {
    List<cav_msgs.ExternalObject> objects = externalObjects.getObjects();
    List<RoadwayObstacle> roadwayObstacles = new LinkedList<>();
    Transform earthToOdom = roadwayMgr.getTransform(earthFrame, odomFrame, externalObjects.getHeader().getStamp());
    for (cav_msgs.ExternalObject obj: objects) {
      roadwayObstacles.add(buildObstacleFromMsg(obj, earthToOdom));
    }
    // publish roadwayObstacles as new Envrionment Message
  }

  protected RoadwayObstacle buildObstacleFromMsg(cav_msgs.ExternalObject obj, Transform earthToOdom) {
    //Get Id
    int id = obj.getId();  
    // Convert object to ECEF frame  
    Transform objInOdom = Transform.fromPoseMessage(obj.getPose().getPose());
    Transform objInECEF = earthToOdom.multiply(objInOdom);
    Vector3 objVecECEF = objInECEF.getTranslation();
    Point3D objPositionECEF = new Point3D(objVecECEF.getX(), objVecECEF.getY(), objVecECEF.getZ());

    // Find the route segment which this route segment is on
    int currentSegIndex = currentSegment.getUptrackWaypoint().getWaypointId();
    List<RouteSegment> segmentsToSearch = activeRoute.findRouteSubsection(currentSegIndex, distBackward, distForward, routeState.getSegmentDownTrack());
    RouteSegment bestSegment = routeSegmentOfPoint(objPositionECEF, segmentsToSearch);
    int segmentIndex = bestSegment.getUptrackWaypoint().getWaypointId();
   
    // Convert object to segment frame
    Transform objInSegment = bestSegment.getECEFToSegmentTransform().invert().multiply(objInECEF); // Find the transform from the segment to this object
    Vector3 objVec = objInSegment.getTranslation();
    Point3D objPosition = new Point3D(objVec.getX(), objVec.getY(), objVec.getZ());
    double downtrackDistance = objDowntrack(segmentIndex, currentSegIndex, objPosition.getX(), routeState.getSegmentDownTrack(), routeState.getDownTrack());
    double crosstrackDistance = objPosition.getY(); //bestSegment.crossTrackDistance(objPosition);
    
    // Convert velocities
    Vector3D velocityLinear = Vector3D.fromVector(objInSegment.apply(Vector3.fromVector3Message(obj.getVelocity().getTwist().getLinear())));

    // Calculate obj lanes
    int primaryLane = determinePrimaryLane(bestSegment, crosstrackDistance);
    Vector3D[] bounds = getAABB(obj, objInSegment);
    List<Integer>  secondaryLanes = determineSecondaryLanes(bounds[0], bounds[1], primaryLane, bestSegment);
    
    // Convert AABB to size
    double sizeX = (bounds[1].getX() - bounds[0].getX()) / 2.0;
    double sizeY = (bounds[1].getY() - bounds[0].getY()) / 2.0;
    double sizeZ = (bounds[1].getZ() - bounds[0].getZ()) / 2.0;
    Vector3D size = new Vector3D(sizeX, sizeY, sizeZ);

    // Construct new roadway obstacle
    RoadwayObstacle newObstacle = messageFactory.newFromType(RoadwayObstacle._TYPE);
    Obstacle newObstacle = new Obstacle(id, downtrackDistance, crosstrackDistance, velocityLinear, size, primaryLane);
    newObstacle.setSecondaryLanes(secondaryLanes);
    
    return newObstacle;
  }

  protected double objDowntrack(int objSegmentIndex, int hostSegmentIndex, double objSegDowntrack, double hostSegDowntrack, double hostDowntrack) {
    if (objSegmentIndex == hostSegmentIndex) {
      return hostDowntrack + (objSegDowntrack - hostSegDowntrack);
    } else if (objSegmentIndex < hostSegmentIndex) {

      double intermediateDist = activeRoute.lengthOfSegments(objSegmentIndex + 1, hostSegmentIndex - 1);
      double remainingObjSegDist = activeRoute.getSegments().get(objSegmentIndex).length() - objSegDowntrack;
      return hostDowntrack - remainingObjSegDist - intermediateDist - hostSegDowntrack; 
    } else { // objSegmentIndex > hostSegmentIndex

      double intermediateDist = activeRoute.lengthOfSegments(objSegmentIndex + 1, hostSegmentIndex - 1);
      double remainingHostSegDist = activeRoute.getSegments().get(hostSegmentIndex).length() - hostSegDowntrack;
      return hostDowntrack + remainingHostSegDist + intermediateDist + objSegDowntrack; 
    }
  }

  protected RouteSegment routeSegmentOfPoint(Point3D point, List<RouteSegment> segments) {
    int count = 0;
    RouteSegment bestSegment = segments.get(0);
    for (RouteSegment seg: segments) {      
      RouteWaypoint wp = seg.getDowntrackWaypoint();
      double crossTrack = seg.crossTrackDistance(point);
      double downTrack = seg.downTrackDistance(point);

      if (-0.0 < downTrack && downTrack < seg.length()) { 
        if (wp.getMinCrossTrack() < crossTrack && crossTrack < wp.getMaxCrossTrack())
          return seg;
        
        bestSegment = seg;
      } else if (count == segments.size() - 1 && downTrack > seg.length()) {
        bestSegment = seg;
      }
      count++;
    }
    return bestSegment;
  }

  // Non existant lanes will be added based on the 
  // TODO support relative lanes in external object description
  protected int determinePrimaryLane(RouteSegment seg, double crossTrack) {
    int segLane = seg.getDowntrackWaypoint().getLaneIndex();
    double laneWidth = seg.getDowntrackWaypoint().getLaneWidth();
    return (int) ((double)segLane - ((crossTrack - (laneWidth / 2.0)) / laneWidth));
  }

  protected Vector3D[] getAABB(cav_msgs.ExternalObject obj, Transform segmentToObj) {
    geometry_msgs.Vector3 size = obj.getSize();
    // bounding box conversion based off http://dev.theomader.com/transform-bounding-boxes/
    double[][] rotMat = QuaternionUtils.quaternionToMat(segmentToObj.getRotationAndScale());
    Vector3D col1 = new Vector3D(rotMat[0][0], rotMat[1][0], rotMat[2][0]);
    Vector3D col2 = new Vector3D(rotMat[0][0], rotMat[1][0], rotMat[2][0]);
    Vector3D col3 = new Vector3D(rotMat[0][0], rotMat[1][0], rotMat[2][0]);

    Vector3D xa = (Vector3D) col1.scalarMultiply(-size.getX());
    Vector3D xb = (Vector3D) col1.scalarMultiply(size.getX());
 
    Vector3D ya = (Vector3D) col2.scalarMultiply(-size.getY());
    Vector3D yb = (Vector3D) col2.scalarMultiply(size.getY());
 
    Vector3D za = (Vector3D) col3.scalarMultiply(-size.getZ());
    Vector3D zb = (Vector3D) col3.scalarMultiply(size.getZ());
 
    Vector3D translation = Vector3D.fromVector(segmentToObj.getTranslation());

    // Could reduce the number of calculations here since only y values are needed
    Vector3D minBounds = (Vector3D) Vector3D.min(xa, xb).add(Vector3D.min(ya, yb)).add(Vector3D.min(za, zb)).add(translation);
    Vector3D maxBounds = (Vector3D) Vector3D.max(xa, xb).add(Vector3D.max(ya, yb)).add(Vector3D.max(za, zb)).add(translation);

    Vector3D[] bounds = new Vector3D[] {
      minBounds, maxBounds
    };
    return bounds;
  }

  protected List<Integer> determineSecondaryLanes(Vector3D minBounds, Vector3D maxBounds, int primaryLane, RouteSegment seg) {
    int minLane = determinePrimaryLane(seg, minBounds.getY());
    int maxLane = determinePrimaryLane(seg, maxBounds.getY());

    List<Integer> secondaryLanes = new LinkedList<>();
    for (int i = minLane; i <= maxLane; i++) {
      if (i != primaryLane)
        secondaryLanes.add(i);
    }
    return secondaryLanes;
  }

  /**
   * SystemAlert message handler.
   * Will shutdown this node on receipt of FATAL or SHUTDOWN
   *
   * @param alert alert message
   */
  public void handleSystemAlertMsg(cav_msgs.SystemAlert alert) {
    switch (alert.getType()) {
      case SystemAlert.DRIVERS_READY:
        break;
      case SystemAlert.NOT_READY:
        break;
      case SystemAlert.SHUTDOWN:
        log.info("SHUTDOWN", "Received SHUTDOWN on system_alert");
        roadwayMgr.shutdown();
        break;
      case SystemAlert.FATAL:
        log.info("SHUTDOWN", "Received FATAL on system_alert");
        roadwayMgr.shutdown();
        break;
      default:
        // No need to handle other types of alert
    }
  }
}
