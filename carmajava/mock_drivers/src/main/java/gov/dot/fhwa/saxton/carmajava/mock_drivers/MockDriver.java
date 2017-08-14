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

// This class provides a node which mimicks a driver's ros messaging behavior but has no logical functionality.
package gov.dot.fhwa.saxton.carmajava.mock_drivers;

import org.apache.commons.logging.Log;
import org.ros.concurrent.CancellableLoop;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import org.ros.node.parameter.ParameterTree;
import org.ros.node.service.ServiceServer;
import org.ros.node.service.ServiceResponseBuilder;
import org.ros.node.service.ServiceResponseListener;
import org.ros.node.service.ServiceClient;
import org.ros.namespace.NameResolver;
import org.ros.namespace.GraphName;
import org.ros.message.MessageFactory;
import org.ros.message.MessageListener;
import org.ros.exception.RemoteException;
import org.ros.exception.RosRuntimeException;
import org.ros.exception.ServiceNotFoundException;

/**
 * A simple {@link Publisher} {@link NodeMain}.
 * <p>
 * Replace PubSub with the node name on Column D but using CamelCase.
 */
public class MockDriver extends AbstractNodeMain {

  //TODO: Replace with Column D node name
  @Override public GraphName getDefaultNodeName() {
    return GraphName.of("mock_driver");
  }

  @Override public void onStart(final ConnectedNode connectedNode) {
    final Log log = connectedNode.getLog();
    final ParameterTree params = connectedNode.getParameterTree();

    // ROS Messages used by all drivers

    // Topics
    // Published
    final Publisher<cav_msgs.SystemAlert> systemAlertPublisher =
      connectedNode.newPublisher("system_alert", cav_msgs.SystemAlert._TYPE);
    final Publisher<bond.Status> bondPub = connectedNode.newPublisher("~/bond", bond.Status._TYPE);
    final Publisher<cav_msgs.DriverStatus> discoveryPub =
      connectedNode.newPublisher("driver_discovery", cav_msgs.DriverStatus._TYPE);

    // Subscribed
    Subscriber<cav_msgs.SystemAlert> alertSub =
      connectedNode.newSubscriber("system_alert", cav_msgs.SystemAlert._TYPE);

    alertSub.addMessageListener(new MessageListener<cav_msgs.SystemAlert>() {
      @Override public void onNewMessage(cav_msgs.SystemAlert message) {

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

        //TODO: Replace with Column D node name
        log.info(params.getString("~/driver_type") + " heard: \"" + message.getDescription() + ";"
          + messageTypeFullDescription + "\"");

      }//onNewMessage
    });//addMessageListener

    // Service
    // Server
    ServiceServer<cav_srvs.BindRequest, cav_srvs.BindResponse> bindService =
      connectedNode.newServiceServer("~/bind", cav_srvs.Bind._TYPE,
        new ServiceResponseBuilder<cav_srvs.BindRequest, cav_srvs.BindResponse>() {
          @Override public void build(cav_srvs.BindRequest request,
            cav_srvs.BindResponse response) {
          }
        });
    ServiceServer<cav_srvs.GetAPISpecificationRequest, cav_srvs.GetAPISpecificationResponse>
      getApiService = connectedNode
      .newServiceServer("~/get_driver_api", cav_srvs.GetAPISpecification._TYPE,
        new ServiceResponseBuilder<cav_srvs.GetAPISpecificationRequest, cav_srvs.GetAPISpecificationResponse>() {
          @Override public void build(cav_srvs.GetAPISpecificationRequest request,
            cav_srvs.GetAPISpecificationResponse response) {
          }
        });

    // Parameters
    final String rosRunID = params.getString("/run_id");

    // ROS Messages unique to each driver

    switch (params.getString("~/driver_type")) {
      case "srx_can_application":
        // Topics
        // Published
        final Publisher<std_msgs.Bool> accPub =
          connectedNode.newPublisher("~/can/acc_engaged", std_msgs.Bool._TYPE);
        final Publisher<std_msgs.Float64> accelPub =
          connectedNode.newPublisher("~/can/acceleration", std_msgs.Float64._TYPE);
        final Publisher<std_msgs.Bool> break_lightsPub =
          connectedNode.newPublisher("~/can/brake_lights", std_msgs.Bool._TYPE);
        final Publisher<std_msgs.Float64> break_positionPub =
          connectedNode.newPublisher("~/can/brake_position", std_msgs.Float64._TYPE);
        final Publisher<std_msgs.Float64> engine_speedPub =
          connectedNode.newPublisher("~/can/engine_speed", std_msgs.Float64._TYPE);
        final Publisher<std_msgs.Float64> fuel_flowPub =
          connectedNode.newPublisher("~/can/fuel_flow", std_msgs.Float64._TYPE);
        final Publisher<std_msgs.Float64> odometryPub =
          connectedNode.newPublisher("~/can/odometer", std_msgs.Float64._TYPE);
        final Publisher<std_msgs.Bool> parking_brakePub =
          connectedNode.newPublisher("~/can/parking_brake", std_msgs.Bool._TYPE);
        final Publisher<std_msgs.Float64> speedPub =
          connectedNode.newPublisher("~/can/speed", std_msgs.Float64._TYPE);
        final Publisher<std_msgs.Float64> steeringPub =
          connectedNode.newPublisher("~/can/steering_wheel_angle", std_msgs.Float64._TYPE);
        final Publisher<std_msgs.Float64> throttlePub =
          connectedNode.newPublisher("~/can/throttle_position", std_msgs.Float64._TYPE);
//        final Publisher<cav_msgs.TurnSignal> turn_sigPub =
//          connectedNode.newPublisher("~/can/turn_signal_state", cav_msgs.TurnSignal._TYPE);

        // Published Parameter ~/device_port
        // Published Parameter ~/timeout

      case "srx_application":

        // Topics
        // Published
        final Publisher<diagnostic_msgs.DiagnosticArray> diagnosticsPub = connectedNode
          .newPublisher("~/control/diagnostics", diagnostic_msgs.DiagnosticArray._TYPE);
        //        final Publisher<cav_msgs.RobotEnabled> enabledPub =
        //          connectedNode.newPublisher("~/control/robot_enabled", cav_msgs.RobotEnabled._TYPE);

        // Subscribed
        Subscriber<std_msgs.Float32> longEffortSub =
          connectedNode.newSubscriber("~/control/cmd_longitudinal_effort", std_msgs.Float32._TYPE);
        //        Subscriber<cav_msgs.SpeedAccel> subscriber =
        //          connectedNode.newSubscriber("~/control/cmd_speed", cav_msgs.SpeedAccel._TYPE);

        // Services
        // Server
        //        ServiceServer<cav_srvs.GetLightsRequest, cav_srvs.GetLightsResponse> getLightsService =
        //          connectedNode.newServiceServer("~/control/get_lights", cav_srvs.GetLights._TYPE,
        //            new ServiceResponseBuilder<cav_srvs.GetLightsRequest, cav_srvs.GetLightsResponse>() {
        //              @Override public void build(cav_srvs.GetLightsRequest request,
        //                cav_srvs.GetLightsResponse response) {
        //                return response;
        //                return response;
        //              }
        //            });
        //        ServiceServer<cav_srvs.SetLightsRequest, cav_srvs.SetLightsResponse> setLightsService =
        //          connectedNode.newServiceServer("~/control/set_lights", cav_srvs.SetLights._TYPE,
        //            new ServiceResponseBuilder<cav_srvs.SetLightsRequest, cav_srvs.SetLightsResponse>() {
        //              @Override public void build(cav_srvs.SetLightsRequest request,
        //                cav_srvs.SetLightsResponse response) {
        //                return response;
        //              }
        //            });
        //        Published	Parameter	~/device_port
        //        Published	Parameter	~/k_d
        //        Published	Parameter	~/k_i
        //        Published	Parameter	~/k_p

      case "srx_object_application":

        // Topics
        // Published
        final Publisher<cav_msgs.ExternalObjectList> objectPub =
          connectedNode.newPublisher("~/sensor/tracked_objects", cav_msgs.ExternalObjectList._TYPE);

        //        Published	Parameter	~/aoi_angle
        //        Published	Parameter	~/device_port
        //        Published	Parameter	~/min_width
        //        Published	Parameter	~/timeout

      case "arada_application":

        // Topics
        // Published
        final Publisher<cav_msgs.ByteArray> recvPub =
          connectedNode.newPublisher("~/comms/recv", cav_msgs.ByteArray._TYPE);

        // Subscribed
        Subscriber<cav_msgs.ByteArray> outboundPub =
          connectedNode.newSubscriber("~/comms/outbound", cav_msgs.ByteArray._TYPE);

        //        Published	Parameter	~/arada_address
        //        Published	Parameter	~/arada_listening_port
        //        Published	Parameter	~/listening_port
        //        Published	Parameter	~/output_queue_size

      default:
    }

    // This CancellableLoop will be canceled automatically when the node shuts
    // down.
    connectedNode.executeCancellableLoop(new CancellableLoop() {
      private int sequenceNumber;

      @Override protected void setup() {
        sequenceNumber = 0;
      }//setup

      @Override protected void loop() throws InterruptedException {
        cav_msgs.SystemAlert systemAlertMsg = systemAlertPublisher.newMessage();
        systemAlertMsg.setDescription(
          "Hello World! " + "I am " + params.getString("~/driver_type") + ". " + sequenceNumber
            + " run_id = " + rosRunID + ".");
        systemAlertMsg.setType(cav_msgs.SystemAlert.SYSTEM_READY);

        systemAlertPublisher.publish(systemAlertMsg);

        sequenceNumber++;
        Thread.sleep(1000);
      }//loop
    });//executeCancellableLoop

  }//onStart
}//AbstractNodeMain

