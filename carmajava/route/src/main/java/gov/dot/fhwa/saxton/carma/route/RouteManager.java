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

import cav_msgs.SystemAlert;
import cav_srvs.*;
import gov.dot.fhwa.saxton.carma.rosutils.SaxtonBaseNode;
import org.apache.commons.logging.Log;
import org.ros.message.MessageListener;
import org.ros.node.topic.Subscriber;
import org.ros.concurrent.CancellableLoop;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;
import org.ros.node.parameter.ParameterTree;
import org.ros.node.service.ServiceServer;
import org.ros.node.service.ServiceResponseBuilder;
import sensor_msgs.NavSatFix;

/**
 * ROS Node which handles route loading, selection, and tracking for the STOL CARMA platform.
 * <p>
 * <p>
 * Command line test: rosrun carma route gov.dot.fhwa.saxton.carma.route.RouteManager
 * Command line test for the service:
 * rosservice call /get_available_routes
 * rosservice call /set_active_route "routeID: '1'"
 * rostopic pub /system_alert cav_msgs/SystemAlert '{type: 5, description: hello}'
 * rosparam set /route_manager/default_database_path /home/mcconnelms/to13_ws/src/CarmaPlatform/carmajava/route/src/main/test/resources/routefiles
 */
public class RouteManager extends SaxtonBaseNode {

  // Topics
  // Publishers
  Publisher<cav_msgs.SystemAlert> systemAlertPub;
  Publisher<cav_msgs.RouteSegment> segmentPub;
  Publisher<cav_msgs.Route> routePub;
  Publisher<cav_msgs.RouteState> routeStatePub;
  // Subscribers
  Subscriber<sensor_msgs.NavSatFix> gpsSub;
  Subscriber<cav_msgs.SystemAlert> alertSub;
  // Services
  // Provided
  protected ServiceServer<SetActiveRouteRequest, SetActiveRouteResponse> setActiveRouteService;
  protected ServiceServer<GetAvailableRoutesRequest, GetAvailableRoutesResponse>
    getAvailableRouteService;
  protected IRouteWorker routeWorker;

  @Override public GraphName getDefaultNodeName() {
    return GraphName.of("route_manager");
  }

  @Override public void onStart(final ConnectedNode connectedNode) {

    final Log log = connectedNode.getLog();
    // Parameters
    ParameterTree params = connectedNode.getParameterTree();
    routeWorker = new RouteWorker(log, params.getString("~default_database_path"));

    /// Topics
    // Publishers
    systemAlertPub = connectedNode.newPublisher("system_alert", cav_msgs.SystemAlert._TYPE);
    segmentPub = connectedNode.newPublisher("current_segment", cav_msgs.RouteSegment._TYPE);
    routePub = connectedNode.newPublisher("route", cav_msgs.Route._TYPE);
    routeStatePub = connectedNode.newPublisher("route_state", cav_msgs.RouteState._TYPE);

    // Subscribers
    //Subscriber<cav_msgs.Tim> timSub = connectedNode.newSubscriber("tim", cav_msgs.Map._TYPE); //TODO: Add once we have tim messages
    gpsSub = connectedNode.newSubscriber("nav_sat_fix", sensor_msgs.NavSatFix._TYPE);
    gpsSub.addMessageListener(new MessageListener<NavSatFix>() {
      @Override public void onNewMessage(NavSatFix navSatFix) {
        routeWorker.handleNavSatFixMsg(navSatFix);
      }
    });

    alertSub = connectedNode.newSubscriber("system_alert", cav_msgs.SystemAlert._TYPE);
    alertSub.addMessageListener(new MessageListener<cav_msgs.SystemAlert>() {
      @Override public void onNewMessage(cav_msgs.SystemAlert message) {
        routeWorker.handleSystemAlertMsg(message);
      }//onNewMessage
    });//addMessageListener

    // Services
    // Server
    getAvailableRouteService = connectedNode
      .newServiceServer("get_available_routes", GetAvailableRoutes._TYPE,
        new ServiceResponseBuilder<GetAvailableRoutesRequest, GetAvailableRoutesResponse>() {
          @Override public void build(GetAvailableRoutesRequest request,
            GetAvailableRoutesResponse response) {
            response = routeWorker.getAvailableRoutes();
          }
        });

    setActiveRouteService = connectedNode.newServiceServer("set_active_route", SetActiveRoute._TYPE,
      new ServiceResponseBuilder<SetActiveRouteRequest, SetActiveRouteResponse>() {
        @Override
        public void build(SetActiveRouteRequest request, SetActiveRouteResponse response) {
          response = routeWorker.setActiveRoute(request);
        }
      });

    // This CancellableLoop will be canceled automatically when the node shuts down
    connectedNode.executeCancellableLoop(new CancellableLoop() {
      private int sequenceNumber;

      @Override protected void setup() {
        sequenceNumber = 0;
      }//setup

      @Override protected void loop() throws InterruptedException {
        // Publish all queued route worker system alert messages
        for (SystemAlert alert : routeWorker.getSystemAlertTopicMsgs()){
          systemAlertPub.publish(alert);
        }

        // If an active route has been selected then publish the route and current segment
        if (routeWorker.getState() == WorkerState.READY_TO_FOLLOW || routeWorker.getState() == WorkerState.FOLLOWING_ROUTE){
          routePub.publish(routeWorker.getActiveRouteTopicMsg());
        }

        // If following a selected route then publish the route state and current segment
        if (routeWorker.getState() == WorkerState.FOLLOWING_ROUTE){
          routeStatePub.publish(routeWorker.getRouteStateTopicMsg(sequenceNumber, connectedNode.getCurrentTime()));
          segmentPub.publish(routeWorker.getCurrentRouteSegmentTopicMsg());
        }

        sequenceNumber++;
        Thread.sleep(100);
      }
    });
  }//onStart
}//AbstractNodeMain
