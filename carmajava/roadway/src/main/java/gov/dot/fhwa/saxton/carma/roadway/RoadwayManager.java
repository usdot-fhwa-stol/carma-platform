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

import cav_msgs.RoadwayEnvironment;
import cav_srvs.GetTransformRequest;
import cav_srvs.GetTransformResponse;
import gov.dot.fhwa.saxton.carma.rosutils.AlertSeverity;
import gov.dot.fhwa.saxton.carma.rosutils.RosServiceSynchronizer;
import gov.dot.fhwa.saxton.carma.rosutils.SaxtonBaseNode;
import gov.dot.fhwa.saxton.carma.rosutils.SaxtonLogger;
import gov.dot.fhwa.saxton.carma.route.Route;
import gov.dot.fhwa.saxton.carma.route.RouteSegment;
import java.util.LinkedList;
import java.util.List;
import org.ros.exception.RemoteException;
import org.ros.message.MessageFactory;
import org.ros.message.Time;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeConfiguration;
import org.ros.node.parameter.ParameterTree;
import org.ros.node.service.ServiceResponseListener;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import org.ros.node.service.ServiceClient;
import org.ros.rosjava_geometry.Transform;
import tf2_msgs.TFMessage;

/**
 * ROS Node which maintains a description of the roadway geometry and obstacles while the STOL CARMA platform is in operation
 * <p>
 * Command line test: rosrun carma roadway gov.dot.fhwa.saxton.carma.roadway.RoadwayManager
 **/
public class RoadwayManager extends SaxtonBaseNode implements IRoadwayManager{

  protected final NodeConfiguration nodeConfiguration = NodeConfiguration.newPrivate();
  protected final MessageFactory messageFactory = nodeConfiguration.getTopicMessageFactory();
  protected ConnectedNode connectedNode;
  protected EnvironmentWorker environmentWorker;
  protected TransformMaintainer transformMaintainer;
  protected SaxtonLogger log;

  // Publishers
  protected Publisher<tf2_msgs.TFMessage> tfPub;
  protected Publisher<cav_msgs.RoadwayEnvironment> roadwayEnvPub;
  protected Publisher<geometry_msgs.PoseArray> posesPub;
  // Subscribers
  protected Subscriber<cav_msgs.Route> routeSub;
  protected Subscriber<cav_msgs.RouteState> routeStateSub;
  protected Subscriber<cav_msgs.HeadingStamped> headingSub;
  protected Subscriber<sensor_msgs.NavSatFix> gpsSub;
  protected Subscriber<nav_msgs.Odometry> odometrySub;
  protected Subscriber<cav_msgs.ExternalObjectList> objectsSub;
  protected Subscriber<geometry_msgs.TwistStamped> velocitySub;
  protected Subscriber<cav_msgs.SystemAlert> systemAlertSub;
  // Used Services
  protected ServiceClient<cav_srvs.GetTransformRequest, cav_srvs.GetTransformResponse> getTransformClient;

  @Override public GraphName getDefaultNodeName() {
    return GraphName.of("environment_manager");
  }

  @Override public void onSaxtonStart(final ConnectedNode connectedNode) {

    this.log = new SaxtonLogger(this.getClass().getSimpleName(), connectedNode.getLog());
    this.connectedNode = connectedNode;

    // Parameters
    ParameterTree params = connectedNode.getParameterTree();
    String earthFrameId = params.getString("~earth_frame_id", "earth");
    String mapFrameId = params.getString("~map_frame_id", "map");
    String odomFrameId = params.getString("~odom_frame_id", "odom");
    String baseLinkFrameId = params.getString("~base_link_frame_id", "base_link");
    String globalPositionSensorFrameId = params.getString("~position_sensor_frame_id", "pinpoint");
    String localPositionSensorFrameId = params.getString("~local_position_sensor_frame_id", "pinpoint");
    double distBackward = params.getDouble("~distance_behind_vehicle", 100.0);
    double distForward = params.getDouble("~distance_infront_of_vehicle", 200.0);

    // Topics Initialization
    // Publishers
    tfPub = connectedNode.newPublisher("/tf", tf2_msgs.TFMessage._TYPE);

    roadwayEnvPub =
      connectedNode.newPublisher("roadway_environment", cav_msgs.RoadwayEnvironment._TYPE);
    posesPub = connectedNode.newPublisher("/route_poses", geometry_msgs.PoseArray._TYPE);
    // Safer to initialize EnvironmentWorker after publishers and before subscribers
    // This means any future modifications which attempt to publish data shortly after initialization will be valid
    environmentWorker = new EnvironmentWorker(this, connectedNode.getLog(), earthFrameId,
      mapFrameId, odomFrameId, baseLinkFrameId, globalPositionSensorFrameId, localPositionSensorFrameId,
       distBackward, distForward);
    transformMaintainer = new TransformMaintainer(this, connectedNode.getLog(), earthFrameId,
      mapFrameId, odomFrameId, baseLinkFrameId, globalPositionSensorFrameId, localPositionSensorFrameId);

    // Used Services
    // Must be called before message subscribers
    getTransformClient = this.waitForService("get_transform", cav_srvs.GetTransform._TYPE, connectedNode, 8000);
    if (getTransformClient == null) {
      log.fatal("TRANSFORM", "Node could not find service get_transform");
      publishSystemAlert(AlertSeverity.FATAL, "Node could not find service get_transform: get_transform service is not available. Roadway package will not be able to function", null );
    }

    // Subscribers
    //Subscriber<cav_msgs.Map> mapSub = connectedNode.newSubscriber("map", cav_msgs.Map._TYPE);//TODO: Include once Map.msg is created
    routeSub = connectedNode.newSubscriber("route", cav_msgs.Route._TYPE);
      routeSub.addMessageListener((cav_msgs.Route message) -> {
        try {
          Route r = Route.fromMessage(message);
          geometry_msgs.PoseArray array = posesPub.newMessage();
          array.getHeader().setFrameId("earth");
          array.getHeader().setStamp(connectedNode.getCurrentTime());
          List<geometry_msgs.Pose> poses = new LinkedList<>();
          for (RouteSegment seg: r.getSegments()) {
            geometry_msgs.Pose pose = connectedNode.getTopicMessageFactory().newFromType(geometry_msgs.Pose._TYPE);
            pose = seg.getECEFToSegmentTransform().toPoseMessage(pose);
            poses.add(pose);
          }
          array.setPoses(poses);
          posesPub.publish(array);
          environmentWorker.handleRouteMsg(message);
        } catch (Throwable e) {
          handleException(e);
        }
      });//MessageListener

    routeStateSub =
    connectedNode.newSubscriber("route_state", cav_msgs.RouteState._TYPE);
    routeStateSub.addMessageListener((cav_msgs.RouteState message) -> {
        try {
          environmentWorker.handleRouteStateMsg(message);
        } catch (Throwable e) {
          handleException(e);
        }
      });//MessageListener

    headingSub = connectedNode.newSubscriber("heading", cav_msgs.HeadingStamped._TYPE);
    headingSub.addMessageListener((cav_msgs.HeadingStamped message) -> {
        try {
          transformMaintainer.handleHeadingMsg(message);
        } catch (Throwable e) {
          handleException(e);
        }
      });//MessageListener

    gpsSub = connectedNode.newSubscriber("nav_sat_fix", sensor_msgs.NavSatFix._TYPE);
    gpsSub.addMessageListener((sensor_msgs.NavSatFix message) -> {
        try {
          transformMaintainer.handleNavSatFixMsg(message);
        } catch (Throwable e) {
          handleException(e);
        }
      });//MessageListener

    odometrySub = connectedNode.newSubscriber("odometry", nav_msgs.Odometry._TYPE);
    odometrySub.addMessageListener((nav_msgs.Odometry message) -> {
        try {
          transformMaintainer.handleOdometryMsg(message);
        } catch (Throwable e) {
          handleException(e);
        }
      });//MessageListener

    objectsSub = connectedNode.newSubscriber("objects", cav_msgs.ExternalObjectList._TYPE);
    objectsSub.addMessageListener((cav_msgs.ExternalObjectList message) -> {
        try {
          environmentWorker.handleExternalObjectsMsg(message);
        } catch (Throwable e) {
          handleException(e);
        }
      });//MessageListener

    velocitySub = connectedNode.newSubscriber("velocity", geometry_msgs.TwistStamped._TYPE);
    velocitySub.addMessageListener((geometry_msgs.TwistStamped message) -> {
        try {
          transformMaintainer.handleVelocityMsg(message);
        } catch (Throwable e) {
          handleException(e);
        }
      });//MessageListener

    systemAlertSub = connectedNode.newSubscriber("system_alert", cav_msgs.SystemAlert._TYPE);
    systemAlertSub.addMessageListener((cav_msgs.SystemAlert message) -> {
        try {
          environmentWorker.handleSystemAlertMsg(message);
        } catch (Throwable e) {
          handleException(e);
        }
      });//onNewMessage
  }

  /***
   * Handles unhandled exceptions and reports to SystemAlert topic, and log the alert.
   * @param e The exception to handle
   */
  @Override
  protected void handleException(Throwable e) {
    String msg = "Uncaught exception in " + connectedNode.getName() + " caught by handleException";
    publishSystemAlert(AlertSeverity.FATAL, msg, e);
    connectedNode.shutdown();
  }

  @Override public void publishTF(TFMessage tfMessage) {
    tfPub.publish(tfMessage);
  }

  @Override public void publishRoadwayEnvironment(RoadwayEnvironment roadwayEnvMsg) {
    roadwayEnvPub.publish(roadwayEnvMsg);
  }

  /** TODO move to rosutils as this is a duplicate of a function in interface mgr?
   * Helper class to allow communication of non-constant data out of the anonymous inner class
   * defined for the getTransform() method
   */
  protected class ResultHolder <T> {
    private T result;

    void setResult(T res) {
      result = res;
    }

    T getResult() {
      return result;
    }
  }

  @Override public Transform getTransform(String parentFrame, String childFrame, Time stamp) {
    final GetTransformRequest req = getTransformClient.newMessage();
    req.setParentFrame(parentFrame);
    req.setChildFrame(childFrame);
    req.setStamp(stamp);
    final ResultHolder<Transform> rh = new ResultHolder<>();
    try {
      RosServiceSynchronizer.callSync(getTransformClient, req,
        new ServiceResponseListener<GetTransformResponse>() {
          @Override
          public void onSuccess(GetTransformResponse response) {
            if (response.getErrorStatus() == GetTransformResponse.NO_ERROR
              || response.getErrorStatus() == GetTransformResponse.COULD_NOT_EXTRAPOLATE) {

              rh.setResult(Transform.fromTransformMessage(response.getTransform().getTransform()));

            } else {
              log.warn("TRANSFORM", "Attempt to get transform failed with error code: " + response.getErrorStatus());
              rh.setResult(null);
              return;
            }
          }

          @Override
          public void onFailure(RemoteException e) {
            log.warn("TRANSFORM", "getTransform call failed for " + getTransformClient.getName());
            rh.setResult(null);
          }
        });
    } catch (InterruptedException e) {
      log.warn("TRANSFORM", "getTransform call failed for " + getTransformClient.getName());
      rh.setResult(null);
    }
    return rh.getResult();
  }

  @Override public Time getTime() {
    return connectedNode.getCurrentTime();
  }

  @Override public void shutdown() {
    log.info("SHUTDOWN", "Shutting down after call to shutdown function");
    connectedNode.shutdown();
  }
}
