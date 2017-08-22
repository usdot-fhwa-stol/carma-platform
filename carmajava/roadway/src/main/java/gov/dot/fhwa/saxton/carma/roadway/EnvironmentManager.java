/*
 * Copyright (C) 2017 Michael McConnell.
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

import cav_msgs.SystemAlert;
import geometry_msgs.Quaternion;
import geometry_msgs.TransformStamped;
import gov.dot.fhwa.saxton.carma.rosutils.SaxtonBaseNode;
import org.apache.commons.logging.Log;
import org.ros.exception.RosRuntimeException;
import org.ros.message.MessageFactory;
import org.ros.message.MessageListener;
import org.ros.concurrent.CancellableLoop;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeConfiguration;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import org.ros.node.parameter.ParameterTree;
import org.ros.rosjava_geometry.FrameTransform;
import org.ros.rosjava_geometry.Transform;
import org.ros.rosjava_geometry.Vector3;
import org.ros.node.service.ServiceClient;

import java.awt.*;
import java.util.ArrayList;
import java.util.Arrays;

/**
 * ROS Node which maintains a description of the roadway geometry and obstacles while the STOL CARMA platform is in operation
 * <p>
 * <p>
 * Command line test: rosrun carma roadway gov.dot.fhwa.saxton.carma.roadway.EnvironmentManager
 **/
public class EnvironmentManager extends SaxtonBaseNode {

  protected final NodeConfiguration nodeConfiguration = NodeConfiguration.newPrivate();
  protected final MessageFactory messageFactory = nodeConfiguration.getTopicMessageFactory();
  protected ConnectedNode nodeHandle;
  protected boolean systemStarted = false;
  // Publishers
  Publisher<tf2_msgs.TFMessage> tfPub;
  Publisher<cav_msgs.SystemAlert> systemAlertPub;
  Publisher<cav_msgs.RoadwayEnvironment> roadwayEnvPub;
  // Subscribers
  Subscriber<cav_msgs.RouteSegment> routeSegmentSub;
  Subscriber<cav_msgs.HeadingStamped> headingSub;
  Subscriber<sensor_msgs.NavSatFix> gpsSub;
  Subscriber<nav_msgs.Odometry> odometrySub;
  Subscriber<cav_msgs.ExternalObjectList> objectsSub;
  Subscriber<cav_msgs.ConnectedVehicleList> vehiclesSub;
  Subscriber<geometry_msgs.TwistStamped> velocitySub;
  Subscriber<cav_msgs.SystemAlert> systemAlertSub;

  @Override public GraphName getDefaultNodeName() {
    return GraphName.of("environment_manager");
  }

  @Override public void onStart(final ConnectedNode connectedNode) {

    final Log log = connectedNode.getLog();
    nodeHandle = connectedNode;

    // Topics Initialization
    // Publishers
    tfPub = connectedNode.newPublisher("/tf", tf2_msgs.TFMessage._TYPE);
    systemAlertPub = connectedNode.newPublisher("system_alert", cav_msgs.SystemAlert._TYPE);
    roadwayEnvPub =
      connectedNode.newPublisher("roadway_environment", cav_msgs.RoadwayEnvironment._TYPE);

    // Subscribers
    //Subscriber<cav_msgs.Map> mapSub = connectedNode.newSubscriber("map", cav_msgs.Map._TYPE);//TODO: Include once Map.msg is created
    routeSegmentSub =
      connectedNode.newSubscriber("route_current_segment", cav_msgs.RouteSegment._TYPE);
    headingSub = connectedNode.newSubscriber("heading", cav_msgs.HeadingStamped._TYPE);
    gpsSub = connectedNode.newSubscriber("nav_sat_fix", sensor_msgs.NavSatFix._TYPE);
    odometrySub = connectedNode.newSubscriber("odometry", nav_msgs.Odometry._TYPE);
    objectsSub = connectedNode.newSubscriber("tracked_objects", cav_msgs.ExternalObjectList._TYPE);
    vehiclesSub =
      connectedNode.newSubscriber("tracked_vehicles", cav_msgs.ConnectedVehicleList._TYPE);
    velocitySub = connectedNode.newSubscriber("velocity", geometry_msgs.TwistStamped._TYPE);
    systemAlertSub = connectedNode.newSubscriber("system_alert", cav_msgs.SystemAlert._TYPE);

    systemAlertSub.addMessageListener(new MessageListener<cav_msgs.SystemAlert>() {
      @Override public void onNewMessage(cav_msgs.SystemAlert message) {
        switch (message.getType()) {
          case cav_msgs.SystemAlert.SYSTEM_READY:
            systemStarted = true;
            break;
          default:
            break;
        }
      }//onNewMessage
    });//MessageListener

    // Used Services

    ServiceClient<cav_srvs.GetTransformRequest, cav_srvs.GetTransformResponse> getTransformClient =
      this.waitForService("get_transform", cav_srvs.GetTransform._TYPE, connectedNode, 5000);

    if (getTransformClient == null) {
      log.error(connectedNode.getName() + "Node could not find service get_transform");
      //throw new RosRuntimeException(connectedNode.getName() + " Node could not find service get_transform");
    }

    //Getting the ros param called run_id. TODO: Remove after rosnetwork validation
    ParameterTree param = connectedNode.getParameterTree();
    final String rosRunID = param.getString("/run_id");

    // This CancellableLoop will be canceled automatically when the node shuts
    // down.
    connectedNode.executeCancellableLoop(new CancellableLoop() {
      private int sequenceNumber;

      @Override protected void setup() {
        sequenceNumber = 0;
      }

      @Override protected void loop() throws InterruptedException {
        publishTF();
        if (!systemStarted) {
          return;
        }

        sequenceNumber++;
        Thread.sleep(1000);
      }
    });
  }

  protected void publishTF() {
    if (tfPub == null || nodeHandle == null) {
      return;
    }
    tf2_msgs.TFMessage tfMsg = tfPub.newMessage();

    geometry_msgs.TransformStamped mapOdomTF = messageFactory.newFromType(TransformStamped._TYPE);
    mapOdomTF = calcMapOdomTF().toTransformStampedMessage(mapOdomTF);

    geometry_msgs.TransformStamped odomBaseLinkTF = messageFactory.newFromType(TransformStamped._TYPE);
    odomBaseLinkTF = calcOdomBaseLinkTF().toTransformStampedMessage(odomBaseLinkTF);

    tfMsg.setTransforms(new ArrayList<>(Arrays.asList(mapOdomTF, odomBaseLinkTF)));
    tfPub.publish(tfMsg);
  }

  protected FrameTransform calcMapOdomTF() {
    // TODO: Calculate transform instead of using identity transform
    org.ros.rosjava_geometry.Vector3.zero();
    org.ros.rosjava_geometry.Quaternion.identity();
    Transform transform = new Transform(org.ros.rosjava_geometry.Vector3.zero(),
      org.ros.rosjava_geometry.Quaternion.identity());
    FrameTransform mapOdomTF =
      new FrameTransform(transform, GraphName.of("odom"), GraphName.of("map"),
        nodeHandle.getCurrentTime());
    return mapOdomTF;
  }

  protected FrameTransform calcOdomBaseLinkTF() {
    // TODO: Calculate transform instead of using identity transform
    org.ros.rosjava_geometry.Vector3.zero();
    org.ros.rosjava_geometry.Quaternion.identity();
    Transform transform = new Transform(org.ros.rosjava_geometry.Vector3.zero(),
      org.ros.rosjava_geometry.Quaternion.identity());
    FrameTransform odomBaseLinkTF =
      new FrameTransform(transform, GraphName.of("base_link"), GraphName.of("odom"),
        nodeHandle.getCurrentTime());
    return odomBaseLinkTF;
  }

  protected void publishRoadwayEnv() {
    if (roadwayEnvPub == null || nodeHandle == null) {
      return;
    }

  }
}

/*
# A list of other vehicle obstacles on the roadway
    cav_msgs/VehicleObstacle[] otherVehicles

# A list of lane geometries sorted by lane id
    cav_msgs/Lane[] lanes

# A description of the host vehicle geometry
    cav_msgs/VehicleObstacle hostVehicle
 */

//  cav_msgs.SystemAlert systemAlertMsg = systemAlertPub.newMessage();
//        systemAlertMsg.setDescription(
//          "Hello World! " + "I am environment_manager. " + sequenceNumber + " run_id = " + rosRunID
//          + ".");