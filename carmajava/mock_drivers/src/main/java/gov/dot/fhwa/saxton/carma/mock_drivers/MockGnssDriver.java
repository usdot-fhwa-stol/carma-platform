/*
 * Copyright (C) 2018-2020 LEIDOS.
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

import org.ros.message.Time;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * A class which can be used to simulate a GNSS driver for the CARMA Platform.
 * <p>
 * Command line test:
 * ROSJava does not support rosrun parameter setting so a rosrun is a multi step process
 * rosparam set /mock_driver/simulated_driver 'gnss'
 * rosparam set /mock_driver/data_file_path '/opt/carma/test_data/pinpoint_stationary.csv'
 * rosrun carma mock_drivers gov.dot.fhwa.saxton.carma.mock_drivers.MockDriverNode
 */
public class MockGnssDriver extends AbstractMockDriver {

  // Topics
  // Published
  final Publisher<geometry_msgs.PoseWithCovarianceStamped> headingPub;
  final Publisher<sensor_msgs.NavSatFix> navSatFixPub;
  final Publisher<geometry_msgs.TwistWithCovarianceStamped> velocityPub;

  // CONSTANTS
  // TODO update for different message spec
  final short SAMPLE_ID_IDX = 0;
  final short HEADING_IDX = 1;
  final short NAV_SRV_IDX = 2;
  final short NAV_STATUS_IDX = 3;
  final short NAV_LAT_IDX = 4;
  final short NAV_LON_IDX = 5;
  final short NAV_ALT_IDX = 6;
  final short NAV_POS_COVR_TYPE_IDX = 7;
  final short ODOM_TWIST_ANG_X_IDX = 8;
  final short ODOM_TWIST_ANG_Y_IDX = 9;
  final short ODOM_TWIST_ANG_Z_IDX = 10;
  final short ODOM_TWIST_LIN_X_IDX = 11;
  final short ODOM_TWIST_LIN_Y_IDX = 12;
  final short ODOM_TWIST_LIN_Z_IDX = 13;
  final short POINT_X_IDX = 14;
  final short POINT_Y_IDX = 15;
  final short POINT_Z_IDX = 16;
  final short QUAT_W_IDX = 17;
  final short QUAT_X_IDX = 18;
  final short QUAT_Y_IDX = 19;
  final short QUAT_Z_IDX = 20;
  final short VEL_ANG_X_IDX = 21;
  final short VEL_ANG_Y_IDX = 22;
  final short VEL_ANG_Z_IDX = 23;
  final short VEL_LIN_X_IDX = 24;
  final short VEL_LIN_Y_IDX = 25;
  final short VEL_LIN_Z_IDX = 26;
  final short COVARINCE_ELEMENT_COUNT = 36;
  final short POS_COVARINCE_ELEMENT_COUNT = 9;
  final short MIN_POSE_COVAR_IDX = 27;
  final short MIN_ODOM_TWIST_COVAR_IDX = MIN_POSE_COVAR_IDX + POS_COVARINCE_ELEMENT_COUNT;
  final short MIN_ODOM_POSE_COVAR_IDX = MIN_ODOM_TWIST_COVAR_IDX + COVARINCE_ELEMENT_COUNT;
  private final short EXPECTED_DATA_COL_COUNT = MIN_ODOM_POSE_COVAR_IDX + COVARINCE_ELEMENT_COUNT;

  public MockGnssDriver(ConnectedNode connectedNode) {
    super(connectedNode);
    // Topics
    // Published
    headingPub =
      connectedNode.newPublisher("gnss/heading_raw", geometry_msgs.PoseWithCovarianceStamped._TYPE);
    navSatFixPub =
      connectedNode.newPublisher("gnss/fix_raw", sensor_msgs.NavSatFix._TYPE);
    velocityPub =
      connectedNode.newPublisher("gnss/vel_raw", geometry_msgs.TwistWithCovarianceStamped._TYPE);
  }

  @Override protected void publishData(List<String[]> data) throws IllegalArgumentException {

    for (String[] elements : data) {
      // Make messages
      geometry_msgs.PoseWithCovarianceStamped headingMsg = headingPub.newMessage();
      sensor_msgs.NavSatFix navMsg = navSatFixPub.newMessage();
      geometry_msgs.TwistWithCovarianceStamped velocityMsg = velocityPub.newMessage();

      // Get data for headers
      int seq = Integer.parseInt(elements[SAMPLE_ID_IDX]);
      Time time = connectedNode.getCurrentTime();

      // Set Data
      // Build Heading Message
      headingMsg.getHeader().setFrameId("0"); // Heading is deg east of north and therefore does not have a frame
      headingMsg.getHeader().setStamp(time);
      headingMsg.getHeader().setSeq(seq);

      // Build NavSatFix Message
      navMsg.getHeader().setFrameId("pinpoint");
      navMsg.getHeader().setSeq(seq);
      navMsg.getHeader().setStamp(time);

      sensor_msgs.NavSatStatus navSatStatus = navMsg.getStatus();
      navSatStatus.setService(Short.parseShort(elements[NAV_SRV_IDX]));
      navSatStatus.setStatus(Byte.parseByte(elements[NAV_STATUS_IDX]));
      navMsg.setStatus(navSatStatus);

      navMsg.setLatitude(Double.parseDouble(elements[NAV_LAT_IDX]));
      navMsg.setLongitude(Double.parseDouble(elements[NAV_LON_IDX]));
      navMsg.setAltitude(Double.parseDouble(elements[NAV_ALT_IDX]));
      navMsg.setPositionCovarianceType(Byte.parseByte(elements[NAV_POS_COVR_TYPE_IDX]));

      double[] posCovariance = new double[POS_COVARINCE_ELEMENT_COUNT];
      for (int i = 0; i < POS_COVARINCE_ELEMENT_COUNT; i++) {
        posCovariance[i] = Double.parseDouble(elements[MIN_POSE_COVAR_IDX + i]);
      }
      navMsg.setPositionCovariance(posCovariance);

      // Build Velocity Message (TwistStamped)
      velocityMsg.getHeader().setFrameId("base_link");
      velocityMsg.getHeader().setSeq(seq);
      velocityMsg.getHeader().setStamp(time);

      geometry_msgs.Twist twist = velocityMsg.getTwist().getTwist();
      geometry_msgs.Vector3 angularVel = messageFactory.newFromType(geometry_msgs.Vector3._TYPE);
      geometry_msgs.Vector3 linearVel = messageFactory.newFromType(geometry_msgs.Vector3._TYPE);
      angularVel.setX(Double.parseDouble(elements[VEL_ANG_X_IDX]));
      angularVel.setY(Double.parseDouble(elements[VEL_ANG_Y_IDX]));
      angularVel.setZ(Double.parseDouble(elements[VEL_ANG_Z_IDX]));

      linearVel.setX(Double.parseDouble(elements[VEL_LIN_X_IDX]));
      linearVel.setY(Double.parseDouble(elements[VEL_LIN_Y_IDX]));
      linearVel.setZ(Double.parseDouble(elements[VEL_LIN_Z_IDX]));

      twist.setAngular(angularVel);
      twist.setLinear(linearVel);
      velocityMsg.getTwist().setTwist(twist);

      // Publish Data
      headingPub.publish(headingMsg);
      navSatFixPub.publish(navMsg);
      velocityPub.publish(velocityMsg);
    }
  }

  @Override protected short getExpectedColCount() {
    return EXPECTED_DATA_COL_COUNT;
  }

  @Override protected short getSampleIdIdx(){
    return SAMPLE_ID_IDX;
  }

  @Override protected List<String> getDriverTypesList() {
    return new ArrayList<>(Arrays.asList("gnss"));
  }

  @Override public List<String> getDriverAPI() {
    return new ArrayList<>(Arrays.asList(
      "gnss/heading_raw",
      "gnss/fix_raw",
      "gnss/vel_raw"));
  }
}
