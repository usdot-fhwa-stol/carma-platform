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
    return GraphName.of("environment_manager");
  }

  @Override
  public void onStart(final ConnectedNode connectedNode) {
    // Topics
    // Publishers
    final Publisher<tf2_msgs.TFMessage> tf_pub =
      connectedNode.newPublisher("/tf", tf2_msgs.TFMessage._TYPE);
    final Publisher<cav_msgs.SystemAlert> system_alert_pub =
      connectedNode.newPublisher("system_alert", cav_msgs.SystemAlert._TYPE);
    final Publisher<cav_msgs.RoadwayEnvironment> roadway_env_pub =
      connectedNode.newPublisher("roadway_environment", cav_msgs.RoadwayEnvironment._TYPE);
    // Subscribers
    Subscriber<cav_msgs.Map> map_sub = connectedNode.newSubscriber("map", cav_msgs.Map._TYPE);
    Subscriber<cav_msgs.RouteSegment> route_segment_sub = connectedNode.newSubscriber("route_current_segment", cav_msgs.RouteSegment._TYPE);
    Subscriber<cav_msgs.HeadingStamped> heading_sub = connectedNode.newSubscriber("heading", cav_msgs.HeadingStamped._TYPE);
    Subscriber<sensor_msgs.NavSatFix> gps_sub = connectedNode.newSubscriber("nav_sat_fix", sensor_msgs.NavSatFix._TYPE);
    Subscriber<nav_msgs.Odometry> odometry_sub = connectedNode.newSubscriber("odometry", nav_msgs.Odometry._TYPE);
    Subscriber<cav_msgs.ExternalObjectList> objects_sub = connectedNode.newSubscriber("tracked_objects", cav_msgs.ExternalObjectList._TYPE);
    Subscriber<cav_msgs.ConnectedVehicleList> vehicles_sub = connectedNode.newSubscriber("tracked_vehicles", cav_msgs.ConnectedVehicleList._TYPE);
    Subscriber<geometry_msgs.TwistStamped> velocity_sub = connectedNode.newSubscriber("velocity", geometry_msgs.TwistStamped._TYPE);
    Subscriber<cav_msgs.SystemAlert> alert_sub = connectedNode.newSubscriber("system_alert", cav_msgs.SystemAlert._TYPE);
    alert_sub.addMessageListener(new MessageListener<cav_msgs.SystemAlert>() {
      @Override
      public void onNewMessage(cav_msgs.SystemAlert alertMsg) {
        log.info("SystemAlert: \"" + alertMsg.getData() + "\"");
      }
    });

    // Used Services
    ServiceClient<cav_srvs.GetTransformRequest, cav_srvs.GetTransformResponse> getTransformClient =
      connectedNode.newServiceClient("get_transform", cav_srvs.GetTransform._TYPE);

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
        // !!! MODIFY FOR SYSTEM ALERT
        std_msgs.String str = publisher.newMessage();
        str.setData("Hello world! " + sequenceNumber);
        publisher.publish(str);
        Thread.sleep(1000);
      }
    });
  }
}
