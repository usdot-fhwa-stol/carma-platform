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

import CarmaPlatform.carmajava.mock_drivers.src.main.java.gov.dot.fhwa.saxton.carma.mock_drivers.AbstractMockDriver;
import org.apache.commons.logging.Log;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;
import org.ros.namespace.GraphName;
import java.util.Arrays;
import java.util.List;

/**
 * A class which can be used to simulate a CAN driver for the CarmaPlatform.
 * <p>
 * Command line test:
 * rosrun carmajava mock_drivers gov.dot.fhwa.saxton.carma.mock_drivers.MockCANDriver
 */
public class MockCANDriver extends AbstractMockDriver {

  final Publisher<std_msgs.Bool> accPub;
  final Publisher<std_msgs.Float64> accelPub;
  final Publisher<std_msgs.Bool> breakLightsPub;
  final Publisher<std_msgs.Float64> breakPositionPub;
  final Publisher<std_msgs.Float64> engineSpeedPub;
  final Publisher<std_msgs.Float64> fuelFlowPub;
  final Publisher<std_msgs.Float64> odometryPub;
  final Publisher<std_msgs.Bool> parkingBrakePub;
  final Publisher<std_msgs.Float64> speedPub;
  final Publisher<std_msgs.Float64> steeringPub;
  final Publisher<std_msgs.Float64> throttlePub;
  final Publisher<cav_msgs.TurnSignal> turnSignalPub;

  final int expectedDataRowCount = 12;

  public MockCANDriver(ConnectedNode connectedNode) {
    super(connectedNode);
    // Topics
    // Published
    accPub = connectedNode.newPublisher("~/can/acc_engaged", std_msgs.Bool._TYPE);
    accelPub = connectedNode.newPublisher("~/can/acceleration", std_msgs.Float64._TYPE);
    breakLightsPub = connectedNode.newPublisher("~/can/brake_lights", std_msgs.Bool._TYPE);
    breakPositionPub = connectedNode.newPublisher("~/can/brake_position", std_msgs.Float64._TYPE);
    engineSpeedPub = connectedNode.newPublisher("~/can/engine_speed", std_msgs.Float64._TYPE);
    fuelFlowPub = connectedNode.newPublisher("~/can/fuel_flow", std_msgs.Float64._TYPE);
    odometryPub = connectedNode.newPublisher("~/can/odometer", std_msgs.Float64._TYPE);
    parkingBrakePub = connectedNode.newPublisher("~/can/parking_brake", std_msgs.Bool._TYPE);
    speedPub = connectedNode.newPublisher("~/can/speed", std_msgs.Float64._TYPE);
    steeringPub = connectedNode.newPublisher("~/can/steering_wheel_angle", std_msgs.Float64._TYPE);
    throttlePub = connectedNode.newPublisher("~/can/throttle_position", std_msgs.Float64._TYPE);
    turnSignalPub = connectedNode.newPublisher("~/can/turn_signal_state", cav_msgs.TurnSignal._TYPE);

    // Parameters
    // Published Parameter ~/device_port
    // Published Parameter ~/timeout
  }

  @Override public GraphName getDefaultNodeName() {
    return GraphName.of("mock_can_driver");
  }

  @Override protected void publishData(String[] data) throws IllegalArgumentException {
    if (data.length != expectedDataRowCount) {
      throw new IllegalArgumentException(
        "Publish data called for MockCanDriver with incorrect number of data elements. "
          + "The required number of data elements is " + expectedDataRowCount);
    }
    // Make messages
    std_msgs.Bool acc = accPub.newMessage();
    std_msgs.Float64 accel = accelPub.newMessage();
    std_msgs.Bool breakLights = breakLightsPub.newMessage();
    std_msgs.Float64 breakPos = breakPositionPub.newMessage();
    std_msgs.Float64 engineSpeed = engineSpeedPub.newMessage();
    std_msgs.Float64 fuelFlow = fuelFlowPub.newMessage();
    std_msgs.Float64 odometry = odometryPub.newMessage();
    std_msgs.Bool parkingBrake = parkingBrakePub.newMessage();
    std_msgs.Float64 speed = speedPub.newMessage();
    std_msgs.Float64 steering = steeringPub.newMessage();
    std_msgs.Float64 throttle = throttlePub.newMessage();
    cav_msgs.TurnSignal turnSignalState = turnSignalPub.newMessage();

    // Set Data
    acc.data = Boolean.parseBoolean(canData[0]);
    accel.data = Float.parseFloat(canData[1]);
    breakLights.data = Boolean.parseBoolean(canData[2]);
    breakPos.data = Float.parseFloat(canData[3]);
    engineSpeed.data = Float.parseFloat(canData[4]);
    fuelFlow.data = Float.parseFloat(canData[5]);
    odometry.data = Float.parseFloat(canData[6]);
    parkingBrake.data = Boolean.parseBoolean(canData[7]);
    speed.data = Float.parseFloat(canData[8]);
    steering.data = Float.parseFloat(canData[9]);
    throttle.data = Float.parseFloat(canData[10]);
    turnSignalState.state = Integer.parseInt(canData[11]);

    // Publish Data
    accPub.publish(acc);
    accelPub.publish(accel);
    breakLightsPub.publish(breakLights);
    breakPositionPub.publish(breakPos);
    engineSpeedPub.publish(engineSpeed);
    fuelFlowPub.publish(fuelFlow);
    odometryPub.publish(odometry);
    parkingBrakePub.publish(parkingBrake);
    speedPub.publish(speed);
    steeringPub.publish(steering);
    throttlePub.publish(throttle);
    turnSigPub.publish(turnSignalState);
  }

  @Override protected int getExpectedRowCount() {
    return expectedDataRowCount;
  }

  @Override protected List<String> getDriverTypesList(){
    return Arrays.asList("can");
  }
}
