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

import geometry_msgs.TransformStamped;
import org.apache.commons.logging.Log;
import org.ros.RosCore;
import org.ros.message.MessageListener;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;
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

  @Override public GraphName getDefaultNodeName() {
    return GraphName.of("transform_server");
  }

  @Override public void onSaxtonStart(final ConnectedNode connectedNode) {

    final Log log = connectedNode.getLog();
    final FrameTransformTree tfTree = new FrameTransformTree();
    final NodeConfiguration nodeConfiguration = NodeConfiguration.newPrivate();
    final MessageFactory messageFactory = nodeConfiguration.getTopicMessageFactory();

    //Topics
    // Subscribers
    Subscriber<tf2_msgs.TFMessage> tf_sub =
      connectedNode.newSubscriber("/tf", tf2_msgs.TFMessage._TYPE);
    tf_sub.addMessageListener(new MessageListener<TFMessage>() {
      @Override public void onNewMessage(TFMessage tfMessage) {
        for (TransformStamped transform : tfMessage.getTransforms()) {
          // Add new transform to internal tree
          tfTree.update(transform);
        }
      }
    });

    Subscriber<tf2_msgs.TFMessage> tf_static_sub =
      connectedNode.newSubscriber("/tf_static", tf2_msgs.TFMessage._TYPE);
    tf_static_sub.addMessageListener(new MessageListener<TFMessage>() {
      @Override public void onNewMessage(TFMessage tfMessage) {
        for (TransformStamped transform : tfMessage.getTransforms()) {
          // Add new transform to internal tree
          tfTree.update(transform);
        }
      }
    });

    // Services
    // Server
    ServiceServer<cav_srvs.GetTransformRequest, cav_srvs.GetTransformResponse> transformServer =
      connectedNode.newServiceServer("get_transform", cav_srvs.GetTransform._TYPE,
        new ServiceResponseBuilder<cav_srvs.GetTransformRequest, cav_srvs.GetTransformResponse>() {
          @Override public void build(cav_srvs.GetTransformRequest request,
            cav_srvs.GetTransformResponse response) {
            // Calculate transform between provided frames and return result
            // Rosjava frame transform tree has an reversed concept of source and target
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
          }
        });
  }

  @Override protected void handleException(Exception e) {

  }
}
