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

import org.ros.concurrent.CancellableLoop;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceServer;
import org.ros.node.service.ServiceResponseBuilder;
import org.ros.node.service.ServiceResponseListener;
import org.ros.exception.RemoteException;
import org.ros.exception.RosRuntimeException;
import org.ros.exception.ServiceNotFoundException;

/**
 *
 */
public class TransformServer extends AbstractNodeMain {

  @Override
  public GraphName getDefaultNodeName() {
    return GraphName.of("transform_server");
  }

  @Override
  public void onStart(final ConnectedNode connectedNode) {
    // Topics
    // Subscribers
    Subscriber<tf2_msgs.TFMessage> tf_sub = connectedNode.newSubscriber("/tf", tf2_msgs.TFMessage._TYPE);
    Subscriber<cav_msgs.SystemAlert> alert_sub = connectedNode.newSubscriber("system_alert", cav_msgs.SystemAlert._TYPE);
    alert_sub.addMessageListener(new MessageListener<cav_msgs.SystemAlert>() {
      @Override
      public void onNewMessage(cav_msgs.SystemAlert alertMsg) {
        log.info("SystemAlert: \"" + alertMsg.getData() + "\"");
      }
    });

    // Services
    // Server
    ServiceServer<cav_srvs.GetTransformRequest, cav_srvs.GetTransformResponse> transformServer =
      connectedNode.newServiceServer("get_transform", cav_srvs.GetTransform._TYPE,
        new ServiceResponseBuilder<cav_srvs.GetTransformRequest, cav_srvs.GetTransformResponse>() {
          @Override
          public void build(cav_srvs.GetTransformRequest request, cav_srvs.GetTransformResponse response) {
            return response;
          }
        });

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
        Thread.sleep(1000);
      }
    });
  }
}
