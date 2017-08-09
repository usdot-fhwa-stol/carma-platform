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

package gov.dot.fhwa.saxton.carmajava.route;

import org.apache.commons.logging.Log;
import org.ros.message.MessageListener;
import org.ros.node.topic.Subscriber;

import org.ros.concurrent.CancellableLoop;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;

/**
 * ROS Node which handles route loading,selection, and tracking for the STOL CARMA platform.
 * <p>
 *
 */
public class RouteManager extends AbstractNodeMain {

  @Override public GraphName getDefaultNodeName() {
    return GraphName.of("route_manager");
  }

  @Override public void onStart(final ConnectedNode connectedNode) {

    final Log log = connectedNode.getLog();

    /// Topics
    // Publishers
    final Publisher<cav_msgs.SystemAlert> system_alert_pub =
      connectedNode.newPublisher("system_alert", cav_msgs.SystemAlert._TYPE);
    final Publisher<cav_msgs.RouteSegment> segment_pub =
      connectedNode.newPublisher("current_segment", cav_msgs.RouteSegment._TYPE);
    final Publisher<cav_msgs.Route> route_pub =
      connectedNode.newPublisher("route", cav_msgs.Route._TYPE);
    final Publisher<cav_msgs.RouteState> route_state_pub =
      connectedNode.newPublisher("route_state", cav_msgs.RouteState._TYPE);

    // Subscribers
    Subscriber<cav_msgs.Tim> tim_sub = connectedNode.newSubscriber("tim", cav_msgs.Map._TYPE);
    Subscriber<sensor_msgs.NavSatFix> gps_sub =
      connectedNode.newSubscriber("nav_sat_fix", sensor_msgs.NavSatFix._TYPE);
    Subscriber<cav_msgs.SystemAlert> alert_sub = connectedNode.newSubscriber("system_alert", cav_msgs.SystemAlert._TYPE);
    subscriber.addMessageListener(new MessageListener<cav_msgs.SystemAlert>() {
      @Override
      public void onNewMessage(cav_msgs.SystemAlert alertMsg) {
        log.info("RouteManager heard system alert: \"" + alertMsg.getData() + "\"");
      }
    });

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
    ParameterTree params = connectedNode.newParameterTree();

    // This CancellableLoop will be canceled automatically when the node shuts
    // down.
    connectedNode.executeCancellableLoop(new CancellableLoop() {
      private int sequenceNumber;
      @Override protected void setup() {
        sequenceNumber = 0;
      }//setup

      @Override protected void loop() throws InterruptedException {
        cav_msgs.SystemAlert alertMsg = system_alert_pub.newMessage();
        alertMsg.setData("RouteManager providing system alert");
        system_alert_pub.publish(alertMsg);

        log.info("RouteManager DatabasePath Param" + params.getString("~/default_database_path"))
        Thread.sleep(1000);
      }
    }//CancellableLoop
    );//executeCancellableLoop
  }//onStart
}//AbstractNodeMain

