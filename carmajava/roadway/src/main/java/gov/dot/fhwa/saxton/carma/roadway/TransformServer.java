/*
 * TODO Copyright (C) 2017 LEIDOS.
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
import geometry_msgs.TransformStamped;
import gov.dot.fhwa.saxton.carma.rosutils.SaxtonLogger;
import org.apache.commons.logging.Log;
import org.ros.RosCore;
import org.ros.message.MessageListener;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;
import org.ros.node.topic.Publisher;
import org.ros.rosjava_geometry.FrameTransform;
import org.ros.rosjava_geometry.FrameTransformTree;
import org.ros.message.MessageFactory;
import gov.dot.fhwa.saxton.carma.rosutils.SaxtonBaseNode;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Subscriber;
import org.ros.node.service.ServiceServer;
import org.ros.node.service.ServiceResponseBuilder;
import tf2_msgs.TFMessage;

/**
 * ROS Node which maintains a tf2 transform tree which can be accessed by other nodes which do not maintain internal trees.
 * The get_transform service can be used to obtain coordinate transformations between two frames
 * <p>
 * Command line test: rosrun carma roadway gov.dot.fhwa.saxton.carma.roadway.TransformServer
 */
public class TransformServer extends SaxtonBaseNode {

  private ConnectedNode connectedNode;
  private SaxtonLogger log;
  //Topics
  // Publishers
  private Publisher<SystemAlert> systemAlertPub;
  // Subscribers
  private Subscriber<cav_msgs.SystemAlert> systemAlertSub;
  private Subscriber<tf2_msgs.TFMessage> tf_sub;
  private Subscriber<tf2_msgs.TFMessage> tf_static_sub;

  // Services
  // Server
  private ServiceServer<cav_srvs.GetTransformRequest, cav_srvs.GetTransformResponse> transformServer;

  @Override public GraphName getDefaultNodeName() {
    return GraphName.of("transform_server");
  }

  @Override public void onSaxtonStart(final ConnectedNode connectedNode) {
    this.connectedNode = connectedNode;
    this.log = new SaxtonLogger(this.getClass().getSimpleName(), connectedNode.getLog());
    final FrameTransformTree tfTree = new FrameTransformTree();
    final NodeConfiguration nodeConfiguration = NodeConfiguration.newPrivate();
    final MessageFactory messageFactory = nodeConfiguration.getTopicMessageFactory();

    //Topics
    // Publishers
    systemAlertPub = connectedNode.newPublisher("system_alert", SystemAlert._TYPE);

    // Subscribers
    systemAlertSub = connectedNode.newSubscriber("system_alert", SystemAlert._TYPE);
    systemAlertSub.addMessageListener(new MessageListener<SystemAlert>() {
      @Override public void onNewMessage(SystemAlert alertMsg) {
        try {
          switch (alertMsg.getType()) {
            case SystemAlert.SHUTDOWN:
              log.logInfo("SHUTDOWN", "Shutting down from SHUTDOWN on system_alert");
              connectedNode.shutdown();
              break;
            case SystemAlert.FATAL:
              log.logInfo("SHUTDOWN", "Shutting down from FATAL on system_alert");
              connectedNode.shutdown();
              break;
            default:
              // No action needed for other types of system alert
          }
        } catch (Throwable e) {
          handleException(e);
        }
      }
    });

    tf_sub = connectedNode.newSubscriber("/tf", tf2_msgs.TFMessage._TYPE);
    tf_sub.addMessageListener(new MessageListener<TFMessage>() {
      @Override public void onNewMessage(TFMessage tfMessage) {
        try {
          for (TransformStamped transform : tfMessage.getTransforms()) {
            // Add new transform to internal tree
            tfTree.update(transform);
          }
        } catch (Throwable e) {
          handleException(e);
        }
      }
    });

    tf_static_sub = connectedNode.newSubscriber("/tf_static", tf2_msgs.TFMessage._TYPE);
    tf_static_sub.addMessageListener(new MessageListener<TFMessage>() {
      @Override public void onNewMessage(TFMessage tfMessage) {
        try {
          for (TransformStamped transform : tfMessage.getTransforms()) {
            // Add new transform to internal tree
            tfTree.update(transform);
          }
        } catch (Throwable e) {
          handleException(e);
        }
      }
    });



    // Services
    // Server
    transformServer = connectedNode.newServiceServer("get_transform", cav_srvs.GetTransform._TYPE,
        new ServiceResponseBuilder<cav_srvs.GetTransformRequest, cav_srvs.GetTransformResponse>() {
          @Override public void build(cav_srvs.GetTransformRequest request,
            cav_srvs.GetTransformResponse response) {
            try {
              // Calculate transform between provided frames and return result
              // RosJava frame transform tree has an reversed concept of source and target
              FrameTransform transform =
                tfTree.transform(request.getChildFrame(), request.getParentFrame());
              geometry_msgs.TransformStamped transformMsg =
                messageFactory.newFromType(TransformStamped._TYPE);
              if (transform != null) {
                transformMsg = transform.toTransformStampedMessage(transformMsg);
                response.setErrorStatus(response.NO_ERROR);
              } else {
                response.setErrorStatus(response.NO_TRANSFORM_EXISTS);
              }
              response.setTransform(transformMsg);
            } catch (Throwable e) {
              handleException(e);
            }
          }
        });
  }

  @Override protected void handleException(Throwable e) {
    String msg = "Uncaught exception made it to top level handleException method";
    log.logFatal("SHUTDOWN", msg, e);
    SystemAlert alertMsg = systemAlertPub.newMessage();
    alertMsg.setType(SystemAlert.FATAL);
    alertMsg.setDescription(msg);
    systemAlertPub.publish(alertMsg);
    connectedNode.shutdown();
  }
}
