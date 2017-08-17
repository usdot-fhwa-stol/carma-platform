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
package gov.dot.fhwa.saxton.carma.mock_drivers;

import CarmaPlatform.carmajava.mock_drivers.src.main.java.gov.dot.fhwa.saxton.carma.mock_drivers.IMockDriver;
import org.apache.commons.logging.Log;
import org.ros.concurrent.CancellableLoop;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.parameter.ParameterTree;
import org.ros.namespace.GraphName;

/**
 * A class which can be used to mimick different drivers for the CarmaPlatform.
 * For specific types of drivers this class should be extended.
 * <p>
 * Command line test:
 * ROSJava does not support rosrun parameter setting so a rosrun is a two step process
 * rosparam set /mock_driver/simulated_driver 'can'
 * rosrun carmajava mock_drivers gov.dot.fhwa.saxton.carma.mock_drivers.MockDriverNode
 */
public class MockDriverNode extends AbstractNodeMain {
  private String defaultName =  "mock_driver";

  @Override public GraphName getDefaultNodeName() {
    return GraphName.of(defaultName);
  }

  @Override public void onStart(final ConnectedNode connectedNode) {
    final Log log = connectedNode.getLog();
    final ParameterTree params = connectedNode.getParameterTree();
    final IMockDriver simulatedDriver;
    switch (params.getString("~/simulated_driver")) {
      case "can":
        simulatedDriver = new MockCANDriver(connectedNode);
        break;
      case "arada":
        simulatedDriver = new MockAradaDriver(connectedNode);
        break;
      case "srx_controller":
        simulatedDriver = new MockSRXControllerDriver(connectedNode);
        break;
      case "radar":
        simulatedDriver = new MockRadarDriver(connectedNode);
        break;
      case "4g":
        simulatedDriver = new MockCellularDriver(connectedNode);
        break;
      case "pinpoint":
        simulatedDriver = new MockPinPointDriver(connectedNode);
        break;
      default:
        log.warn(
          "No valid driver name specified on the simulated_driver parameter. Defaulting to CAN driver");
        simulatedDriver = new MockCANDriver(connectedNode);
        break;
    }

    // This CancellableLoop will be canceled automatically when the node shuts
    // down.
    connectedNode.executeCancellableLoop(new CancellableLoop() {
      private int sequenceNumber;

      @Override protected void setup() {
        sequenceNumber = 0;
      }//setup

      @Override protected void loop() throws InterruptedException {
        simulatedDriver.publishDriverStatus();
        simulatedDriver.readAndPublishData();

        sequenceNumber++;
        Thread.sleep(1000);
      }//loop
    });//executeCancellableLoop

  }//onStart
}//AbstractNodeMain

/*
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
        // TODO: Uncomment once TurnSignal.msg has been implemented
        //final Publisher<cav_msgs.TurnSignal> turn_sigPub =
        //  connectedNode.newPublisher("~/can/turn_signal_state", cav_msgs.TurnSignal._TYPE);

        // Parameters
        // Published Parameter ~/device_port
        // Published Parameter ~/timeout
        break;

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
        // TODO: Uncomment once SpeedAccel.msg is implemented
        // Subscriber<cav_msgs.SpeedAccel> subscriber =
        //   connectedNode.newSubscriber("~/control/cmd_speed", cav_msgs.SpeedAccel._TYPE);

        // Services
        // Server
        // TODO: Uncomment once GetLights and SetLights services are specified.
        //  ServiceServer<cav_srvs.GetLightsRequest, cav_srvs.GetLightsResponse> getLightsService =
        //    connectedNode.newServiceServer("~/control/get_lights", cav_srvs.GetLights._TYPE,
        //      new ServiceResponseBuilder<cav_srvs.GetLightsRequest, cav_srvs.GetLightsResponse>() {
        //        @Override public void build(cav_srvs.GetLightsRequest request,
        //          cav_srvs.GetLightsResponse response) {
        //          return response;
        //          return response;
        //        }
        //      });
        //  ServiceServer<cav_srvs.SetLightsRequest, cav_srvs.SetLightsResponse> setLightsService =
        //    connectedNode.newServiceServer("~/control/set_lights", cav_srvs.SetLights._TYPE,
        //      new ServiceResponseBuilder<cav_srvs.SetLightsRequest, cav_srvs.SetLightsResponse>() {
        //        @Override public void build(cav_srvs.SetLightsRequest request,
        //          cav_srvs.SetLightsResponse response) {
        //          return response;
        //        }
        //      });
        //  Published	Parameter	~/device_port
        //  Published	Parameter	~/k_d
        //  Published	Parameter	~/k_i
        //  Published	Parameter	~/k_p
        break;

      case "srx_object_application":

        // Topics
        // Published
        final Publisher<cav_msgs.ExternalObjectList> objectPub =
          connectedNode.newPublisher("~/sensor/tracked_objects", cav_msgs.ExternalObjectList._TYPE);

        // Published	Parameter	~/aoi_angle
        // Published	Parameter	~/device_port
        // Published	Parameter	~/min_width
        // Published	Parameter	~/timeout
        break;

      case "arada_application":

        // Topics
        // Published
        final Publisher<cav_msgs.ByteArray> recvPub =
          connectedNode.newPublisher("~/comms/recv", cav_msgs.ByteArray._TYPE);

        // Subscribed
        Subscriber<cav_msgs.ByteArray> outboundPub =
          connectedNode.newSubscriber("~/comms/outbound", cav_msgs.ByteArray._TYPE);

        // Published	Parameter	~/arada_address
        // Published	Parameter	~/arada_listening_port
        // Published	Parameter	~/listening_port
        // Published	Parameter	~/output_queue_size

        break;

      default:
    }
 */
