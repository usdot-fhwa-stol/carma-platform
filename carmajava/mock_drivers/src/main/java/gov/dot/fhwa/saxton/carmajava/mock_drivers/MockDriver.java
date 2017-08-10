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
    ParameterTree param = connectedNode.getParameterTree();
    // Topics
    // Published
    switch(param.getString("~/driver_type")){
      case "can":
        final Publisher<bond.Status> bond_pub = connectedNode.newPublisher("~/bond", bond.Status._TYPE);
        final Publisher<std_msgs.Bool> acc_pub =
          connectedNode.newPublisher("~/can/acc_engaged", std_msgs.Bool._TYPE);
        final Publisher<std_msgs.Float64> accel_pub =
          connectedNode.newPublisher("~/can/acceleration", std_msgs.Float64._TYPE);
        final Publisher<std_msgs.Bool> break_lights_pub =
          connectedNode.newPublisher("~/can/brake_lights", std_msgs.Bool._TYPE);
        final Publisher<std_msgs.Float64> break_position_pub =
          connectedNode.newPublisher("~/can/brake_position", std_msgs.Float64._TYPE);
        final Publisher<std_msgs.Float64> engine_speed_pub =
          connectedNode.newPublisher("~/can/engine_speed", std_msgs.Float64._TYPE);
        final Publisher<std_msgs.Float64> fuel_flow_pub =
          connectedNode.newPublisher("~/can/fuel_flow", std_msgs.Float64._TYPE);
        final Publisher<std_msgs.Float64> odometry_pub =
          connectedNode.newPublisher("~/can/odometer", std_msgs.Float64._TYPE);
        final Publisher<std_msgs.Bool> parking_brake_pub =
          connectedNode.newPublisher("~/can/parking_brake", std_msgs.Bool._TYPE);
        final Publisher<std_msgs.Float64> speed_pub =
          connectedNode.newPublisher("~/can/speed", std_msgs.Float64._TYPE);
        final Publisher<std_msgs.Float64> steering_pub =
          connectedNode.newPublisher("~/can/steering_wheel_angle", std_msgs.Float64._TYPE);
        final Publisher<std_msgs.Float64> throttle_pub =
          connectedNode.newPublisher("~/can/throttle_position", std_msgs.Float64._TYPE);
        final Publisher<cav_msgs.TurnSignal> turn_sig_pub =
          connectedNode.newPublisher("~/can/turn_signal_state", cav_msgs.TurnSignal._TYPE);
        final Publisher<cav_msgs.DriverStatus> throttle_pub =
          connectedNode.newPublisher("driver_discovery", cav_msgs.DriverStatus._TYPE);

        // Services
        // Server
        ServiceServer<cav_srvs.BindWithIDRequest, cav_srvs.BindWithIDResponse> bindService =
          connectedNode.newServiceServer("~/bind", cav_srvs.BindWithID._TYPE,
            new ServiceResponseBuilder<cav_srvs.BindWithIDRequest, cav_srvs.BindWithIDResponse>() {
              @Override public void build(cav_srvs.BindWithIDRequest request,
                cav_srvs.BindWithIDResponse response) {
                return response;
              }
            });
        ServiceServer<cav_srvs.GetDriverApiRequest, cav_srvs.GetDriverApiResponse> bindService =
          connectedNode.newServiceServer("~/get_driver_api", cav_srvs.GetDriverApi._TYPE,
            new ServiceResponseBuilder<cav_srvs.GetDriverApiRequest, cav_srvs.GetDriverApiResponse>() {
              @Override public void build(cav_srvs.GetDriverApiRequest request,
                cav_srvs.GetDriverApiResponse response) {
                return response;
              }
            });
        // Published Parameter ~/device_port
        // Published Parameter ~/timeout
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
      //                                             std_msgs.String str = publisher.newMessage();
      //                                             //TODO: Replace with Column D node name
      //                                             str.setData("Hello World! " + "I am pub_sub. " + sequenceNumber);
      //                                             publisher.publish(str);
      //                                             sequenceNumber++;
       Thread.sleep(1000);
      }//loop

                                         }//CancellableLoop
    );//executeCancellableLoop

  }//onStart
}//AbstractNodeMain

