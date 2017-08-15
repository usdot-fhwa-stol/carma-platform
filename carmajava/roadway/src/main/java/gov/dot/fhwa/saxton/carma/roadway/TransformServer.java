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

import org.apache.commons.logging.Log;
import org.ros.message.MessageListener;
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

/**
 * ROS Node which maintains a tf2 transform tree which can be accessed by other nodes which do not maintain internal trees.
 * The get_transform service can be used to optain coordinate transformations between two frames
 * <p>
 *
 *   Command line test: rosrun carma roadway gov.dot.fhwa.saxton.carma.roadway.TransformServer
 */
public class TransformServer extends AbstractNodeMain {

  @Override
  public GraphName getDefaultNodeName() {
    return GraphName.of("transform_server");
  }

  @Override
  public void onStart(final ConnectedNode connectedNode) {

    final Log log = connectedNode.getLog();

    // Topics
    // Publishers
    final Publisher<cav_msgs.SystemAlert> systemAlertPub =
      connectedNode.newPublisher("system_alert", cav_msgs.SystemAlert._TYPE);

    // Subscribers
    Subscriber<tf2_msgs.TFMessage> tf_sub = connectedNode.newSubscriber("/tf", tf2_msgs.TFMessage._TYPE);
    Subscriber<cav_msgs.SystemAlert> systemAlertSub = connectedNode.newSubscriber("system_alert", cav_msgs.SystemAlert._TYPE);
    systemAlertSub.addMessageListener(new MessageListener<cav_msgs.SystemAlert>() {
      @Override
      public void onNewMessage(cav_msgs.SystemAlert message) {

        String messageTypeFullDescription = "NA";

        switch (message.getType()) {
          case cav_msgs.SystemAlert.CAUTION:
            messageTypeFullDescription = "Take caution! ";
            break;
          case cav_msgs.SystemAlert.WARNING:
            messageTypeFullDescription = "I have a warning! ";
            break;
          case cav_msgs.SystemAlert.FATAL:
            messageTypeFullDescription = "I am FATAL! ";
            break;
          case cav_msgs.SystemAlert.NOT_READY:
            messageTypeFullDescription = "I am NOT Ready! ";
            break;
          case cav_msgs.SystemAlert.SYSTEM_READY:
            messageTypeFullDescription = "I am Ready! ";
            break;
          default:
            messageTypeFullDescription = "I am NOT Ready! ";
        }

        log.info("transform_server heard: \"" + message.getDescription() + ";" + messageTypeFullDescription + "\"");

      }//onNewMessage
    });//MessageListener

    // Services
    // Server
    ServiceServer<cav_srvs.GetTransformRequest, cav_srvs.GetTransformResponse> transformServer =
      connectedNode.newServiceServer("get_transform", cav_srvs.GetTransform._TYPE,
        new ServiceResponseBuilder<cav_srvs.GetTransformRequest, cav_srvs.GetTransformResponse>() {
          @Override
          public void build(cav_srvs.GetTransformRequest request, cav_srvs.GetTransformResponse response) {
          }
        });

    //Getting the ros param called run_id. TODO: Remove after rosnetwork validation
    ParameterTree param = connectedNode.getParameterTree();
    final String rosRunID = param.getString("/run_id");

    // This CancellableLoop will be canceled automatically when the node shuts
    // down.
    connectedNode.executeCancellableLoop(new CancellableLoop() {
      private int sequenceNumber;

      @Override
      protected void setup() {
        sequenceNumber = 0;
      }

      @Override
      protected void loop() throws InterruptedException {
        cav_msgs.SystemAlert systemAlertMsg = systemAlertPub.newMessage();
        systemAlertMsg.setDescription("Hello World! " + "I am transform_server. " + sequenceNumber + " run_id = " + rosRunID + ".");
        systemAlertMsg.setType(cav_msgs.SystemAlert.SYSTEM_READY);

        systemAlertPub.publish(systemAlertMsg);
        sequenceNumber++;
        Thread.sleep(30000);
      }
    });
  }
}
