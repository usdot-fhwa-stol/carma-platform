/*
 * Copyright (C) 2018-2019 LEIDOS.
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
 * A class which can be used to simulate a pinpoint position driver for the CarmaPlatform.
 * <p>
 * Command line test:
 * ROSJava does not support rosrun parameter setting so a rosrun is a multi step process
 * rosparam set /mock_driver/simulated_driver 'pinpoint'
 * rosparam set /mock_driver/data_file_path '/opt/carma/test_data/pinpoint_stationary.csv'
 * rosrun carma mock_drivers gov.dot.fhwa.saxton.carma.mock_drivers.MockDriverNode
 */
public class MockPinPointDriver extends AbstractMockDriver {

  // Topics
  // Published
  final Publisher<cav_msgs.HeadingStamped> headingPub;
  final Publisher<sensor_msgs.NavSatFix> navSatFixPub;
  final Publisher<nav_msgs.Odometry> odometryPub;
  final Publisher<geometry_msgs.TwistStamped> velocityPub;

  // CONSTANTS
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

  public MockPinPointDriver(ConnectedNode connectedNode) {
    super(connectedNode);
    // Topics
    // Published
    headingPub =
      connectedNode.newPublisher("position/heading", cav_msgs.HeadingStamped._TYPE);
    navSatFixPub =
      connectedNode.newPublisher("position/nav_sat_fix", sensor_msgs.NavSatFix._TYPE);
    odometryPub =
      connectedNode.newPublisher("position/odometry", nav_msgs.Odometry._TYPE);
    velocityPub =
      connectedNode.newPublisher("position/velocity", geometry_msgs.TwistStamped._TYPE);
  }

  @Override protected void publishData(List<String[]> data) throws IllegalArgumentException {

    for (String[] elements : data) {
      // Make messages
      cav_msgs.HeadingStamped headingMsg = headingPub.newMessage();
      sensor_msgs.NavSatFix navMsg = navSatFixPub.newMessage();
      nav_msgs.Odometry odometryMsg = odometryPub.newMessage();
      geometry_msgs.TwistStamped velocityMsg = velocityPub.newMessage();

      // Get data for headers
      int seq = Integer.parseInt(elements[SAMPLE_ID_IDX]);
      Time time = connectedNode.getCurrentTime();

      // Set Data
      // Build Heading Message
      headingMsg.getHeader().setFrameId("0"); // Heading is deg east of north and therefore does not have a frame
      headingMsg.getHeader().setStamp(time);
      headingMsg.getHeader().setSeq(seq);
      headingMsg.setHeading(Float.parseFloat(elements[HEADING_IDX]));

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

      // Build Odometry Message
      odometryMsg.getHeader().setFrameId("odom");
      odometryMsg.getHeader().setSeq(seq);
      odometryMsg.getHeader().setStamp(time);
      odometryMsg.setChildFrameId("base_link");
      // Odom Twist
      geometry_msgs.TwistWithCovariance odomTwistWithCovar = odometryMsg.getTwist();
      geometry_msgs.Twist odomTwist = odomTwistWithCovar.getTwist();
      geometry_msgs.Vector3 odomAngularVel = odomTwist.getAngular();
      geometry_msgs.Vector3 odomLinearVel = odomTwist.getLinear();
      odomAngularVel.setX(Double.parseDouble(elements[ODOM_TWIST_ANG_X_IDX]));
      odomAngularVel.setY(Double.parseDouble(elements[ODOM_TWIST_ANG_Y_IDX]));
      odomAngularVel.setZ(Double.parseDouble(elements[ODOM_TWIST_ANG_Z_IDX]));

      odomLinearVel.setX(Double.parseDouble(elements[ODOM_TWIST_LIN_X_IDX]));
      odomLinearVel.setY(Double.parseDouble(elements[ODOM_TWIST_LIN_Y_IDX]));
      odomLinearVel.setZ(Double.parseDouble(elements[ODOM_TWIST_LIN_Z_IDX]));

      odomTwist.setAngular(odomAngularVel);
      odomTwist.setLinear(odomLinearVel);
      odomTwistWithCovar.setTwist(odomTwist);

      double[] odomTwistCovariance = new double[COVARINCE_ELEMENT_COUNT];
      for (int i = 0; i < COVARINCE_ELEMENT_COUNT; i++) {
        odomTwistCovariance[i] = Double.parseDouble(elements[MIN_ODOM_TWIST_COVAR_IDX + i]);
      }

      odomTwistWithCovar.setCovariance(odomTwistCovariance);
      odometryMsg.setTwist(odomTwistWithCovar);
      // Odom Pose
      geometry_msgs.PoseWithCovariance poseWithCovar = odometryMsg.getPose();
      geometry_msgs.Pose pose = poseWithCovar.getPose();
      geometry_msgs.Quaternion quat = pose.getOrientation();
      geometry_msgs.Point point = pose.getPosition();
      point.setX(Double.parseDouble(elements[POINT_X_IDX]));
      point.setY(Double.parseDouble(elements[POINT_Y_IDX]));
      point.setZ(Double.parseDouble(elements[POINT_Z_IDX]));
      pose.setPosition(point);

      quat.setW(Double.parseDouble(elements[QUAT_W_IDX]));
      quat.setX(Double.parseDouble(elements[QUAT_X_IDX]));
      quat.setY(Double.parseDouble(elements[QUAT_Y_IDX]));
      quat.setZ(Double.parseDouble(elements[QUAT_Z_IDX]));
      pose.setOrientation(quat);

      poseWithCovar.setPose(pose);

      double[] odomPoseCovariance = new double[COVARINCE_ELEMENT_COUNT];
      for (int i = 0; i < COVARINCE_ELEMENT_COUNT; i++) {
        odomPoseCovariance[i] = Double.parseDouble(elements[MIN_ODOM_POSE_COVAR_IDX + i]);
      }

      poseWithCovar.setCovariance(odomPoseCovariance);
      odometryMsg.setPose(poseWithCovar);

      // Build Velocity Message (TwistStamped)
      velocityMsg.getHeader().setFrameId("base_link");
      velocityMsg.getHeader().setSeq(seq);
      velocityMsg.getHeader().setStamp(time);

      geometry_msgs.Twist twist = velocityMsg.getTwist();
      geometry_msgs.Vector3 angularVel = odomTwist.getAngular();
      geometry_msgs.Vector3 linearVel = odomTwist.getLinear();
      angularVel.setX(Double.parseDouble(elements[VEL_ANG_X_IDX]));
      angularVel.setY(Double.parseDouble(elements[VEL_ANG_Y_IDX]));
      angularVel.setZ(Double.parseDouble(elements[VEL_ANG_Z_IDX]));

      linearVel.setX(Double.parseDouble(elements[VEL_LIN_X_IDX]));
      linearVel.setY(Double.parseDouble(elements[VEL_LIN_Y_IDX]));
      linearVel.setZ(Double.parseDouble(elements[VEL_LIN_Z_IDX]));

      twist.setAngular(angularVel);
      twist.setLinear(linearVel);
      velocityMsg.setTwist(twist);

      // Publish Data
      headingPub.publish(headingMsg);
      navSatFixPub.publish(navMsg);
      odometryPub.publish(odometryMsg);
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
    return new ArrayList<>(Arrays.asList("position"));
  }

  @Override public List<String> getDriverAPI() {
    return new ArrayList<>(Arrays.asList(
      "position/heading",
      "position/nav_sat_fix",
      "position/odometry",
      "position/velocity"));
  }
}
