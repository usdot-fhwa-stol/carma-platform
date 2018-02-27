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

package gov.dot.fhwa.saxton.carma.roadway;

import cav_msgs.RoadwayObstacle;
import cav_msgs.RouteState;
import cav_msgs.SystemAlert;
import geometry_msgs.TransformStamped;
import gov.dot.fhwa.saxton.carma.geometry.GeodesicCartesianConverter;
import gov.dot.fhwa.saxton.carma.geometry.cartesian.CartesianObject;
import gov.dot.fhwa.saxton.carma.geometry.cartesian.Point3D;
import gov.dot.fhwa.saxton.carma.geometry.cartesian.QuaternionUtils;
import gov.dot.fhwa.saxton.carma.geometry.cartesian.Vector;
import gov.dot.fhwa.saxton.carma.geometry.cartesian.Vector3D;
import gov.dot.fhwa.saxton.carma.geometry.geodesic.Location;
import gov.dot.fhwa.saxton.carma.rosutils.SaxtonLogger;
import gov.dot.fhwa.saxton.carma.route.Route;
import gov.dot.fhwa.saxton.carma.route.RouteSegment;
import gov.dot.fhwa.saxton.carma.route.RouteWaypoint;
import gov.dot.fhwa.saxton.carma.route.WorkerState;
import org.apache.commons.lang.ArrayUtils;
import org.apache.commons.logging.Log;
import org.jboss.netty.buffer.ChannelBuffers;
import org.ros.message.Duration;
import org.ros.message.MessageFactory;
import org.ros.message.Time;
import org.ros.node.NodeConfiguration;
import org.ros.rosjava_geometry.Quaternion;
import org.ros.rosjava_geometry.Transform;
import org.ros.rosjava_geometry.Vector3;
import std_msgs.Header;
import tf2_msgs.TFMessage;
import java.nio.ByteOrder;
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
    for (RouteSegment seg : activeRoute.getSegments()) {
      seg.getDowntrackWaypoint().getLocation().setAltitude(170);
      seg.getUptrackWaypoint().getLocation().setAltitude(170);
    }
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
    if (currentSegment == null || routeState == null || activeRoute == null) {
      log.info("Roadway ignoring object message as no route is selected");
      return;
    }
    List<cav_msgs.ExternalObject> objects = externalObjects.getObjects();
    List<RoadwayObstacle> roadwayObstacles = new LinkedList<>();
    Transform earthToOdom = roadwayMgr.getTransform(earthFrame, odomFrame, externalObjects.getHeader().getStamp());
    if (earthToOdom == null) {
      log.warn("Roadway could not process object message as earth to odom transform was null");
    }
    for (cav_msgs.ExternalObject obj: objects) {
      roadwayObstacles.add(buildObstacleFromMsg(obj, earthToOdom));
    }
    cav_msgs.RoadwayEnvironment roadwayMsg = messageFactory.newFromType(cav_msgs.RoadwayEnvironment._TYPE);
    roadwayMsg.setRoadwayObstacles(roadwayObstacles);
    roadwayMgr.publishRoadwayEnvironment(roadwayMsg);
  }

  /**
   * Helper function constructs a RoadwayObstacle from an ExternalObject 
   * 
   * @param obj The external object to convert (Should be defined relative to odom frame)
   * @param earthToOdom The corresponding transform from the earth to the odom frame
   * 
   * @return A fully constructed RoadwayObstacle object
   */
  protected RoadwayObstacle buildObstacleFromMsg(cav_msgs.ExternalObject obj, Transform earthToOdom) {
    //Get Id
    int id = obj.getId();  
    // Get connected vehicle type
    ConnectedVehicleType connectedVehicleType = ConnectedVehicleType.NOT_CONNECTED;
    if ((short) (obj.getPresenceVector() & cav_msgs.ExternalObject.BSM_ID_PRESENCE_VECTOR) != 0) {
      connectedVehicleType = ConnectedVehicleType.CONNECTED;
    }
    // Convert object to ECEF frame  
    Transform objInOdom = Transform.fromPoseMessage(obj.getPose().getPose());
    Transform objInECEF = earthToOdom.multiply(objInOdom);
    Vector3 objVecECEF = objInECEF.getTranslation();
    Point3D objPositionECEF = new Point3D(objVecECEF.getX(), objVecECEF.getY(), objVecECEF.getZ());

    // Find the route segment which this route segment is on
    int currentSegIndex = currentSegment.getUptrackWaypoint().getWaypointId();
    List<RouteSegment> segmentsToSearch = activeRoute.findRouteSubsection(currentSegIndex, routeState.getSegmentDownTrack(), distBackward, distForward);
    RouteSegment bestSegment = activeRoute.routeSegmentOfPoint(objPositionECEF, segmentsToSearch);
    int segmentIndex = bestSegment.getUptrackWaypoint().getWaypointId();
   
    // Convert object to segment frame
    Transform objInSegment = bestSegment.getECEFToSegmentTransform().invert().multiply(objInECEF); // Find the transform from the segment to this object
    Vector3 objVec = objInSegment.getTranslation();
    Point3D objPosition = new Point3D(objVec.getX(), objVec.getY(), objVec.getZ());
    double downtrackDistance = objDowntrack(segmentIndex, currentSegIndex, objPosition.getX(), routeState.getSegmentDownTrack(), routeState.getDownTrack());
    double crosstrackDistance = objPosition.getY(); //bestSegment.crossTrackDistance(objPosition);
    
    // Convert velocities
    Vector3 velocityLinear = objInSegment.apply(Vector3.fromVector3Message(obj.getVelocity().getTwist().getLinear()));

    // Calculate obj lanes
    int primaryLane = bestSegment.determinePrimaryLane(crosstrackDistance);
    // If the relative lane field is defined use that instead of calculated lane
    if ((short) (obj.getPresenceVector() & cav_msgs.ExternalObject.RELATIVE_LANE_PRESENCE_VECTOR) != 0) {
      int expectedLane = primaryLane;
      switch(obj.getRelativeLane()) {
        case cav_msgs.ExternalObject.HOST_LANE:
          expectedLane = routeState.getLaneIndex();
          break;
        case cav_msgs.ExternalObject.RIGHT_LANE:
          expectedLane = routeState.getLaneIndex() - 1;
          break;
        case cav_msgs.ExternalObject.LEFT_LANE:
          expectedLane = routeState.getLaneIndex() + 1;
          break;
      }
      primaryLane = expectedLane;
    }
    // Determine secondary lanes
    List<Point3D> objPoints = new LinkedList<>();
    geometry_msgs.Vector3 size = obj.getSize();
    objPoints.add(new Point3D(size.getX(), size.getY(), size.getZ()));
    objPoints.add(new Point3D(size.getX(), size.getY(),-size.getZ()));
    objPoints.add(new Point3D(size.getX(), -size.getY(), size.getZ()));
    objPoints.add(new Point3D(size.getX(), -size.getY(), -size.getZ()));
    objPoints.add(new Point3D(-size.getX(), size.getY(), size.getZ()));
    objPoints.add(new Point3D(-size.getX(), size.getY(), -size.getZ()));
    objPoints.add(new Point3D(-size.getX(), -size.getY(), size.getZ()));
    objPoints.add(new Point3D(-size.getX(), -size.getY(), -size.getZ()));

    CartesianObject cartObj = new CartesianObject(objPoints);
    CartesianObject cartObjInSegment = cartObj.transform(objInSegment);
    final int minIdx = CartesianObject.MIN_BOUND_IDX;
    final int maxIdx = CartesianObject.MAX_BOUND_IDX;
    final int xIdx = 0;
    final int yIdx = 1;
    final int zIdx = 2;
    double[][] bounds = cartObjInSegment.getBounds();
    byte[]  secondaryLanes = bestSegment.determineSecondaryLanes(bounds[yIdx][minIdx], bounds[yIdx][maxIdx], primaryLane);
    
    // Convert AABB to size
    double sizeX = (bounds[xIdx][maxIdx] - bounds[xIdx][minIdx]) / 2.0;
    double sizeY = (bounds[yIdx][maxIdx] - bounds[yIdx][minIdx]) / 2.0;
    double sizeZ = (bounds[zIdx][maxIdx] - bounds[zIdx][minIdx]) / 2.0;
    geometry_msgs.Vector3 sizeMsg = messageFactory.newFromType(geometry_msgs.Vector3._TYPE);
    sizeMsg.setX(sizeX);
    sizeMsg.setY(sizeY);
    sizeMsg.setZ(sizeZ);

    // Construct new roadway obstacle

    RoadwayObstacle newObstacle = messageFactory.newFromType(RoadwayObstacle._TYPE);
    newObstacle.setConnectedVehicleType(connectedVehicleType.toMessage());
    newObstacle.setCrossTrack(crosstrackDistance);
    newObstacle.setDownTrack(downtrackDistance);
    newObstacle.setPrimaryLane((byte)primaryLane);
    if (secondaryLanes.length > 0) { // Ensure we only try to set if secondary lanes are present
      newObstacle.setSecondaryLanes(ChannelBuffers.copiedBuffer(ByteOrder.LITTLE_ENDIAN, secondaryLanes));
    }
    newObstacle.setWaypointId(bestSegment.getDowntrackWaypoint().getWaypointId());

    cav_msgs.ExternalObject newObj = newObstacle.getObject();
    if (obj.getBsmId().hasArray() && obj.getBsmId().readable() && obj.getBsmId().array().length > 0) {
      newObj.setBsmId(obj.getBsmId());
    }
    newObj.setConfidence(obj.getConfidence());
    newObj.setHeader(obj.getHeader());
    newObj.getHeader().setFrameId("0"); // Uses a route segment specific frame id which is not on the frame transform tree
    newObj.setId(id);
    
    newObj.setObjectType(obj.getObjectType());

    newObj.getPose().setCovariance(obj.getPose().getCovariance());
    newObj.getPose().getPose().getPosition().setX(objPosition.getX());
    newObj.getPose().getPose().getPosition().setY(objPosition.getY());
    newObj.getPose().getPose().getPosition().setZ(objPosition.getZ());
    newObj.getPose().getPose().setOrientation(Quaternion.identity().toQuaternionMessage(newObj.getPose().getPose().getOrientation()));
   
    newObj.setRelativeLane(obj.getRelativeLane());
    newObj.setSize(sizeMsg);
    
    newObj.getVelocity().setCovariance(obj.getVelocity().getCovariance());
    newObj.getVelocity().getTwist().setLinear(velocityLinear.toVector3Message(newObj.getVelocity().getTwist().getLinear()));
    
    // Remove the object parameters which will not be passed on
    newObj.setPresenceVector(
      (short) 
      (obj.getPresenceVector()
      & ~cav_msgs.ExternalObject.AZIMUTH_RATE_PRESENCE_VECTOR
      & ~cav_msgs.ExternalObject.RANGE_RATE_PRESENCE_VECTOR
      & ~cav_msgs.ExternalObject.VELOCITY_INST_PRESENCE_VECTOR)
    );

    newObstacle.setObject(newObj);
    
    return newObstacle;
  }

  /**
   * Calculates the downtrack value of an object based on its segment downtrack
   * 
   * @param objSegmentIndex The index of this objects current segment in the route
   * @param hostSegmentIndex the index of the host vehicle's current segment in the route
   * @param objSegDowntrack the downtrack of the object along its current segment
   * @param hostSegmentDowntrack the downtrack of the host vehicle along its current segment
   * @param hostDowntrack the downtrack of the host vehicle along the route
   * 
   * @return The object's downtrack distance along the entire route
   */
  protected double objDowntrack(int objSegmentIndex, int hostSegmentIndex, double objSegDowntrack, double hostSegDowntrack, double hostDowntrack) {
    if (objSegmentIndex == hostSegmentIndex) {
      
      return hostDowntrack + (objSegDowntrack - hostSegDowntrack);
    } else if (objSegmentIndex < hostSegmentIndex) {

      double intermediateDist = activeRoute.lengthOfSegments(objSegmentIndex + 1, hostSegmentIndex - 1);
      double remainingObjSegDist = activeRoute.getSegments().get(objSegmentIndex).length() - objSegDowntrack;
      return hostDowntrack - remainingObjSegDist - intermediateDist - hostSegDowntrack; 
    } else { // objSegmentIndex > hostSegmentIndex

      double intermediateDist = activeRoute.lengthOfSegments(hostSegmentIndex + 1, objSegmentIndex - 1);
      double remainingHostSegDist = activeRoute.getSegments().get(hostSegmentIndex).length() - hostSegDowntrack;
      return hostDowntrack + remainingHostSegDist + intermediateDist + objSegDowntrack; 
    }
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
