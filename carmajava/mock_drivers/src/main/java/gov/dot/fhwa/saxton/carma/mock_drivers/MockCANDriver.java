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

package gov.dot.fhwa.saxton.carma.mock_drivers;

import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;
import org.ros.namespace.GraphName;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * A class which can be used to simulate a CAN driver for the CarmaPlatform.
 * <p>
 * Command line test:
 * ROSJava does not support rosrun parameter setting so a rosrun is a multi step process
 * rosparam set /mock_driver/simulated_driver 'can'
 * rosparam set /mock_driver/data_file_path '/home/username/temp.csv'
 * rosrun carmajava mock_drivers gov.dot.fhwa.saxton.carma.mock_drivers.MockDriverNode
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

  final short EXPECTED_DATA_COL_COUNT = 13;

  private final short SAMPLE_ID_IDX = 0;
  private final short ACC_IDX = 1;
  private final short ACCEL_IDX = 2;
  private final short BREAK_LIGHTS_IDX = 3;
  private final short BREAK_POS_IDX = 4;
  private final short ENGINE_SPEED_IDX = 5;
  private final short FUEL_FLOW_IDX = 6;
  private final short ODOMETRY_IDX = 7;
  private final short PARKING_BRAKE_IDX = 8;
  private final short SPEED_IDX = 9;
  private final short STEERING_IDX = 10;
  private final short THROTTLE_IDX = 11;
  private final short TURN_SIGNAL_STATE_IDX = 12;

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

  @Override public GraphName getDefaultDriverName() {
    return GraphName.of("mock_can_driver");
  }

  @Override protected void publishData(List<String[]> data) {

    for (String[] elements : data) {
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
      acc.setData(Boolean.parseBoolean(elements[ACC_IDX]));
      accel.setData(Float.parseFloat(elements[ACCEL_IDX]));
      breakLights.setData(Boolean.parseBoolean(elements[BREAK_LIGHTS_IDX]));
      breakPos.setData(Float.parseFloat(elements[BREAK_POS_IDX]));
      engineSpeed.setData(Float.parseFloat(elements[ENGINE_SPEED_IDX]));
      fuelFlow.setData(Float.parseFloat(elements[FUEL_FLOW_IDX]));
      odometry.setData(Float.parseFloat(elements[ODOMETRY_IDX]));
      parkingBrake.setData(Boolean.parseBoolean(elements[PARKING_BRAKE_IDX]));
      speed.setData(Float.parseFloat(elements[SPEED_IDX]));
      steering.setData(Float.parseFloat(elements[STEERING_IDX]));
      throttle.setData(Float.parseFloat(elements[THROTTLE_IDX]));
      turnSignalState.setState(Byte.parseByte(elements[TURN_SIGNAL_STATE_IDX]));

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
      turnSignalPub.publish(turnSignalState);
    }
  }

  @Override protected short getExpectedColCount() {
    return EXPECTED_DATA_COL_COUNT;
  }

  @Override protected short getSampleIdIdx(){
    return SAMPLE_ID_IDX;
  }

  @Override protected List<String> getDriverTypesList(){
    return new ArrayList<>(Arrays.asList("can"));
  }

  @Override public List<String> getDriverAPI() {
    return new ArrayList<>(Arrays.asList(connectedNode.getName() + "/can/acc_engaged",
      connectedNode.getName() + "/can/acceleration", connectedNode.getName() + "/can/brake_lights",
      connectedNode.getName() + "/can/brake_position",
      connectedNode.getName() + "/can/engine_speed", connectedNode.getName() + "/can/fuel_flow",
      connectedNode.getName() + "/can/odometer", connectedNode.getName() + "/can/parking_brake",
      connectedNode.getName() + "/can/speed", connectedNode.getName() + "/can/steering_wheel_angle",
      connectedNode.getName() + "/can/throttle_position",
      connectedNode.getName() + "/can/turn_signal_state"));
  }
}
