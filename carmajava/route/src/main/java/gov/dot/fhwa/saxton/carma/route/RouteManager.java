/*
 * TODO: Copyright (C) 2017 LEIDOS.
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

package gov.dot.fhwa.saxton.carma.route;

import gov.dot.fhwa.saxton.carma.rosutils.SaxtonBaseNode;
import org.apache.commons.logging.Log;
import org.ros.message.MessageListener;
import org.ros.node.topic.Subscriber;
import org.ros.concurrent.CancellableLoop;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.node.parameter.ParameterTree;
import org.ros.namespace.NameResolver;
import org.ros.message.MessageFactory;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceServer;
import org.ros.node.service.ServiceResponseBuilder;
import org.ros.node.service.ServiceResponseListener;
import org.ros.exception.RemoteException;
import org.ros.exception.RosRuntimeException;
import org.ros.exception.ServiceNotFoundException;

/**
 * ROS Node which handles route loading, selection, and tracking for the STOL CARMA platform.
 * <p>
 *
 * Command line test: rosrun carma route gov.dot.fhwa.saxton.carma.route.RouteManager
 */
public class RouteManager extends SaxtonBaseNode {

  @Override public GraphName getDefaultNodeName() {
    return GraphName.of("route_manager");
  }

  @Override public void onStart(final ConnectedNode connectedNode) {

    final Log log = connectedNode.getLog();

    /// Topics
    // Publishers
    final Publisher<cav_msgs.SystemAlert> systemAlertPub =
      connectedNode.newPublisher("system_alert", cav_msgs.SystemAlert._TYPE);
    final Publisher<cav_msgs.RouteSegment> segmentPub =
      connectedNode.newPublisher("current_segment", cav_msgs.RouteSegment._TYPE);
    final Publisher<cav_msgs.Route> routePub =
      connectedNode.newPublisher("route", cav_msgs.Route._TYPE);
    final Publisher<cav_msgs.RouteState> routeStatePub =
      connectedNode.newPublisher("route_state", cav_msgs.RouteState._TYPE);

    // Subscribers
    //Subscriber<cav_msgs.Tim> timSub = connectedNode.newSubscriber("tim", cav_msgs.Map._TYPE); //TODO: Add once we have tim messages
    Subscriber<sensor_msgs.NavSatFix> gpsSub =
      connectedNode.newSubscriber("nav_sat_fix", sensor_msgs.NavSatFix._TYPE);
    Subscriber<cav_msgs.SystemAlert> alertSub = connectedNode.newSubscriber("system_alert", cav_msgs.SystemAlert._TYPE);
    alertSub.addMessageListener(new MessageListener<cav_msgs.SystemAlert>() {
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

        log.info("route_manager heard: \"" + message.getDescription() + ";" + messageTypeFullDescription + "\"");

      }//onNewMessage
    });//addMessageListener

    // Services
    // Server
    ServiceServer<cav_srvs.GetAvailableRoutesRequest, cav_srvs.GetAvailableRoutesResponse>
      getActiveRouteService = connectedNode
      .newServiceServer("get_available_routes", cav_srvs.GetAvailableRoutes._TYPE,
        new ServiceResponseBuilder<cav_srvs.GetAvailableRoutesRequest, cav_srvs.GetAvailableRoutesResponse>() {
          @Override public void build(cav_srvs.GetAvailableRoutesRequest request,
            cav_srvs.GetAvailableRoutesResponse response) {
          }
        });
    ServiceServer<cav_srvs.SetActiveRouteRequest, cav_srvs.SetActiveRouteResponse>
      setActiveRouteService = connectedNode
      .newServiceServer("set_active_route", cav_srvs.SetActiveRoute._TYPE,
        new ServiceResponseBuilder<cav_srvs.SetActiveRouteRequest, cav_srvs.SetActiveRouteResponse>() {
          @Override public void build(cav_srvs.SetActiveRouteRequest request,
            cav_srvs.SetActiveRouteResponse response) {
          }
        });

    // Parameters
    ParameterTree params = connectedNode.getParameterTree();
    //Getting the ros param called run_id.
    final String rosRunID = params.getString("/run_id");
    // This CancellableLoop will be canceled automatically when the node shuts
    // down.

    
    connectedNode.executeCancellableLoop(new CancellableLoop() {
      private int sequenceNumber;
      @Override protected void setup() {
        sequenceNumber = 0;
      }//setup

      @Override protected void loop() throws InterruptedException {
        cav_msgs.SystemAlert systemAlertMsg = systemAlertPub.newMessage();
        systemAlertMsg.setDescription("Hello World! " + "I am route_manager. " + sequenceNumber + " run_id = " + rosRunID + ".");
        systemAlertMsg.setType(cav_msgs.SystemAlert.SYSTEM_READY);

        systemAlertPub.publish(systemAlertMsg);

        //log.info("RouteManager DatabasePath Param" + params.getString("~/default_database_path"))
        sequenceNumber++;
        Thread.sleep(30000);
      }
    }//CancellableLoop
    );//executeCancellableLoop
  }//onStart
}//AbstractNodeMain

