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

import geometry_msgs.TransformStamped;
import org.apache.commons.logging.Log;
import org.ros.message.MessageListener;
import org.ros.node.NodeConfiguration;
import org.ros.rosjava_geometry.FrameTransform;
import org.ros.rosjava_geometry.FrameTransformTree;
import org.ros.message.MessageFactory;
import org.ros.concurrent.CancellableLoop;
import org.ros.namespace.GraphName;
import org.ros.namespace.NameResolver;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceServer;
import org.ros.node.service.ServiceResponseBuilder;
import org.ros.node.service.ServiceResponseListener;
import org.ros.node.parameter.ParameterTree;
import org.ros.exception.RemoteException;
import org.ros.exception.RosRuntimeException;
import org.ros.exception.ServiceNotFoundException;
import tf2_msgs.TFMessage;

/**
 * ROS Node which maintains a tf2 transform tree which can be accessed by other nodes which do not maintain internal trees.
 * The get_transform service can be used to optain coordinate transformations between two frames
 * <p>
 * <p>
 * Command line test: rosrun carma roadway gov.dot.fhwa.saxton.carma.roadway.TransformServer
 */
public class TransformServer extends AbstractNodeMain {
  protected final NodeConfiguration nodeConfiguration = NodeConfiguration.newPrivate();
  protected final MessageFactory messageFactory = nodeConfiguration.getTopicMessageFactory();

  @Override public GraphName getDefaultNodeName() {
    return GraphName.of("transform_server");
  }

  @Override public void onStart(final ConnectedNode connectedNode) {

    final Log log = connectedNode.getLog();
    final FrameTransformTree tfTree = new FrameTransformTree();

    //Topics
    // Subscribers
    Subscriber<tf2_msgs.TFMessage> tf_sub =
      connectedNode.newSubscriber("/tf", tf2_msgs.TFMessage._TYPE);
    tf_sub.addMessageListener(new MessageListener<TFMessage>() {
      @Override public void onNewMessage(TFMessage tfMessage) {
        for (TransformStamped transform : tfMessage.getTransforms()) {
          tfTree.update(transform);
        }
      }
    });

    Subscriber<cav_msgs.SystemAlert> systemAlertSub =
      connectedNode.newSubscriber("system_alert", cav_msgs.SystemAlert._TYPE);
    systemAlertSub.addMessageListener(new MessageListener<cav_msgs.SystemAlert>() {
      @Override public void onNewMessage(cav_msgs.SystemAlert message) {
        switch (message.getType()) {
          case cav_msgs.SystemAlert.CAUTION:
            break; // No need for action
          case cav_msgs.SystemAlert.WARNING:
            break; // No need for action
          case cav_msgs.SystemAlert.FATAL:
            break; // No need for action
          case cav_msgs.SystemAlert.NOT_READY:
            break; // No need for action
          case cav_msgs.SystemAlert.SYSTEM_READY:
            break; // No need for action
          default:
        }
      }//onNewMessage
    });//MessageListener

    // Services
    // Server
    ServiceServer<cav_srvs.GetTransformRequest, cav_srvs.GetTransformResponse> transformServer =
      connectedNode.newServiceServer("get_transform", cav_srvs.GetTransform._TYPE,
        new ServiceResponseBuilder<cav_srvs.GetTransformRequest, cav_srvs.GetTransformResponse>() {
          @Override public void build(cav_srvs.GetTransformRequest request,
            cav_srvs.GetTransformResponse response) {
            FrameTransform transform =
              tfTree.transform(request.getSourceFrame(), request.getTargetFrame());
            geometry_msgs.TransformStamped transformMsg =
              messageFactory.newFromType(TransformStamped._TYPE);
            if (transform != null) {
              transformMsg = transform.toTransformStampedMessage(transformMsg);
              response.setErrorStatus(response.NO_ERROR);
            } else {
              response.setErrorStatus(response.NO_TRANSFORM_EXISTS);
            }
            response.setTransform(transformMsg);
          }
        });

    // This CancellableLoop will be canceled automatically when the node shuts
    // down.
    connectedNode.executeCancellableLoop(new CancellableLoop() {
      private int sequenceNumber;

      @Override protected void setup() {
        sequenceNumber = 0;
      }

      @Override protected void loop() throws InterruptedException {

        sequenceNumber++;
        Thread.sleep(1000);
      }
    });
  }
}
