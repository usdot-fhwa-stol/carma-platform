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
import org.ros.internal.node.service.ServiceManager;
import org.ros.message.*;
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
import org.ros.node.parameter.ParameterTree;
import org.ros.namespace.NameResolver;
import org.ros.message.MessageFactory;

/**
 * ROS Node which maintains a description of the roadway geometry and obstacles while the STOL CARMA platform is in operation
 * <p>
 * <p>
 * Command line test: rosrun carmajava roadway gov.dot.fhwa.saxton.carma.roadway.EnvironmentManager
 **/
public class EnvironmentManager extends AbstractNodeMain {

  @Override public GraphName getDefaultNodeName() {
    return GraphName.of("environment_manager");
  }

  @Override public void onStart(final ConnectedNode connectedNode) {

    final Log log = connectedNode.getLog();

    // Topics
    // Publishers
    final Publisher<tf2_msgs.TFMessage> tfPub =
      connectedNode.newPublisher("/tf", tf2_msgs.TFMessage._TYPE);
    final Publisher<cav_msgs.SystemAlert> systemAlertPub =
      connectedNode.newPublisher("system_alert", cav_msgs.SystemAlert._TYPE);
    final Publisher<cav_msgs.RoadwayEnvironment> roadwayEnvPub =
      connectedNode.newPublisher("roadway_environment", cav_msgs.RoadwayEnvironment._TYPE);

    // Subscribers
    //Subscriber<cav_msgs.Map> mapSub = connectedNode.newSubscriber("map", cav_msgs.Map._TYPE);//TODO: Include once Map.msg is created
    Subscriber<cav_msgs.RouteSegment> routeSegmentSub =
      connectedNode.newSubscriber("route_current_segment", cav_msgs.RouteSegment._TYPE);
    Subscriber<cav_msgs.HeadingStamped> headingSub =
      connectedNode.newSubscriber("heading", cav_msgs.HeadingStamped._TYPE);
    Subscriber<sensor_msgs.NavSatFix> gpsSub =
      connectedNode.newSubscriber("nav_sat_fix", sensor_msgs.NavSatFix._TYPE);
    Subscriber<nav_msgs.Odometry> odometrySub =
      connectedNode.newSubscriber("odometry", nav_msgs.Odometry._TYPE);
    Subscriber<cav_msgs.ExternalObjectList> objectsSub =
      connectedNode.newSubscriber("tracked_objects", cav_msgs.ExternalObjectList._TYPE);
    Subscriber<cav_msgs.ConnectedVehicleList> vehiclesSub =
      connectedNode.newSubscriber("tracked_vehicles", cav_msgs.ConnectedVehicleList._TYPE);
    Subscriber<geometry_msgs.TwistStamped> velocitySub =
      connectedNode.newSubscriber("velocity", geometry_msgs.TwistStamped._TYPE);
    Subscriber<cav_msgs.SystemAlert> systemAlertSub =
      connectedNode.newSubscriber("system_alert", cav_msgs.SystemAlert._TYPE);
    systemAlertSub.addMessageListener(new MessageListener<cav_msgs.SystemAlert>() {
      @Override public void onNewMessage(cav_msgs.SystemAlert message) {
        switch (message.getType()) {
          case cav_msgs.SystemAlert.CAUTION:
            break;
          case cav_msgs.SystemAlert.WARNING:
            break;
          case cav_msgs.SystemAlert.FATAL:
            break;
          case cav_msgs.SystemAlert.NOT_READY:
            break;
          case cav_msgs.SystemAlert.SYSTEM_READY:
            break;
          default:
        }
      }//onNewMessage
    });//MessageListener

    // Used Services
    ServiceClient<cav_srvs.GetTransformRequest, cav_srvs.GetTransformResponse> getTransformClient =
    this.<cav_srvs.GetTransformRequest, cav_srvs.GetTransformResponse>waitForService("get_transform", cav_srvs.GetTransform._TYPE, connectedNode, 5000);

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
        cav_msgs.SystemAlert systemAlertMsg = systemAlertPub.newMessage();
        systemAlertMsg.setDescription(
          "Hello World! " + "I am environment_manager. " + sequenceNumber + " run_id = " + rosRunID
            + ".");
        systemAlertMsg.setType(cav_msgs.SystemAlert.SYSTEM_READY);

        systemAlertPub.publish(systemAlertMsg);
        sequenceNumber++;
        Thread.sleep(30000);
      }
    });
  }

  /**
   * Blocks until the desired service is found and returned or timeout expires. If the timeout expires then returns null.
   *
   * Note: This function should never be called before Definitions of ServiceServers. This will help avoid race conditions.
   *
   * @param service       The name of the ros service
   * @param typeString    The type string defining the service classes. Generally of from std_srvs.SetBool._Type.
   * @param connectedNode The node which is waiting for this service to be available
   * @param timeout       The timeout in milliseconds before this node will cease waiting for this service
   * @param <T>           The service request type such as std_srvs.SetBoolRequest
   * @param <S>           The service response type such as srd_srvs.SetBoolResponse
   * @return An initialized ServiceClient for the desired service
   */
  protected <T, S> ServiceClient<T, S> waitForService(String service, String typeString,
    final ConnectedNode connectedNode, int timeout) {
    ServiceClient<T, S> client = null;
    boolean serviceFound = false;
    Time endTime = connectedNode.getCurrentTime().add(Duration.fromMillis(timeout));
    // Keep searching for service while it is not found and the timeout is not exceeded.
    while (!serviceFound && connectedNode.getCurrentTime().compareTo(endTime) <= 0) {
      try {
        client = connectedNode.newServiceClient(service, typeString);
        serviceFound = true;
      } catch (ServiceNotFoundException e) {
        serviceFound = false;
      }
    }
    return client;
  }
}
