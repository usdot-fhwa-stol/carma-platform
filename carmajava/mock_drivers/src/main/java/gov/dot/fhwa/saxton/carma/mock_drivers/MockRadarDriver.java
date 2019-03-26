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
import java.util.LinkedList;
import java.util.List;

/**
 * A class which can be used to simulate an Radar sensor driver for the CarmaPlatform.
 * <p>
 * Command line test:
 * ROSJava does not support rosrun parameter setting so a rosrun is a multi step process
 * rosparam set /mock_driver/simulated_driver 'radar'
 * rosparam set /mock_driver/data_file_path '/home/username/temp.csv'
 * rosrun carmajava mock_drivers gov.dot.fhwa.saxton.carma.mock_drivers.MockDriverNode
 */
public class MockRadarDriver extends AbstractMockDriver {

  // Topics
  // Published
  Publisher<cav_msgs.ExternalObjectList> flrrObjectPub; // Front long range radar
  Publisher<cav_msgs.ExternalObjectList> lfsrrObjectPub; // Left front short range radar
  Publisher<cav_msgs.ExternalObjectList> rsrrObjectPub; // Rear short range radar
  Publisher<cav_msgs.ExternalObjectList> rfsrrObjectPub; // Right front short range radar
  Publisher<cav_msgs.ExternalObjectList> visionObjectPub; // Camera

  // CONSTANTS
  private final short COVARINCE_ELEMENT_COUNT = 36;
  private final short SAMPLE_ID_IDX = 0;
  private final short ID_IDX = 1;
  private final short SIZE_X_IDX = 2;
  private final short SIZE_Y_IDX = 3;
  private final short SIZE_Z_IDX = 4;
  private final short POINT_X_IDX = 5;
  private final short POINT_Y_IDX = 6;
  private final short POINT_Z_IDX = 7;
  private final short QUAT_W_IDX = 8;
  private final short QUAT_X_IDX = 9;
  private final short QUAT_Y_IDX = 10;
  private final short QUAT_Z_IDX = 11;
  private final short VEL_ANG_X_IDX = 12;
  private final short VEL_ANG_Y_IDX = 13;
  private final short VEL_ANG_Z_IDX = 14;
  private final short VEL_LIN_X_IDX = 15;
  private final short VEL_LIN_Y_IDX = 16;
  private final short VEL_LIN_Z_IDX = 17;
  private final short VEL_INST_ANG_X_IDX = 18;
  private final short VEL_INST_ANG_Y_IDX = 19;
  private final short VEL_INST_ANG_Z_IDX = 20;
  private final short LIN_INST_ANG_X_IDX = 21;
  private final short LIN_INST_ANG_Y_IDX = 22;
  private final short LIN_INST_ANG_Z_IDX = 23;
  private final short MIN_POSE_COVAR_IDX = 24;
  private final short MIN_VEL_COVAR_IDX = MIN_POSE_COVAR_IDX + COVARINCE_ELEMENT_COUNT;
  private final short MIN_VEL_INST_COVAR_IDX = MIN_VEL_COVAR_IDX + COVARINCE_ELEMENT_COUNT;
  private final short EXPECTED_DATA_COL_COUNT = MIN_VEL_INST_COVAR_IDX + COVARINCE_ELEMENT_COUNT;

  /**
   *  Constructor sets up ROS publishers and subscribers
   *
   * @param connectedNode The ROS node which will be used to simulate a Radar Driver
   */
  public MockRadarDriver(ConnectedNode connectedNode) {
    super(connectedNode);
    // Topics
    // Published
    flrrObjectPub = connectedNode.newPublisher("f_lrr/sensor/objects", cav_msgs.ExternalObjectList._TYPE);
    lfsrrObjectPub = connectedNode.newPublisher("lf_srr/sensor/objects", cav_msgs.ExternalObjectList._TYPE);
    rsrrObjectPub = connectedNode.newPublisher("r_srr/sensor/objects", cav_msgs.ExternalObjectList._TYPE);
    rfsrrObjectPub = connectedNode.newPublisher("rf_srr/sensor/objects", cav_msgs.ExternalObjectList._TYPE);
    visionObjectPub = connectedNode.newPublisher("vision/sensor/objects", cav_msgs.ExternalObjectList._TYPE);
  }

  @Override protected void publishData(List<String[]> data) {

    List<cav_msgs.ExternalObject> objects = new LinkedList<>(); // The list of ExternalObjects (obstacles) detected by the radar

    String frameId = "f_lrr_frame";
    Time currentTime = connectedNode.getCurrentTime();
    for (String[] elements :  data){
      cav_msgs.ExternalObject externalObject = messageFactory.newFromType(cav_msgs.ExternalObject._TYPE);

      // Build Header
      std_msgs.Header hdr = externalObject.getHeader();
      hdr.setFrameId(frameId);
      hdr.setSeq(Integer.parseInt(elements[SAMPLE_ID_IDX]));
      hdr.setStamp(currentTime);

      externalObject.setHeader(hdr);
      externalObject.setId(Short.parseShort(elements[ID_IDX]));

      // Build Size Vector
      geometry_msgs.Vector3 size = externalObject.getSize();
      size.setX(Double.parseDouble(elements[SIZE_X_IDX]));
      size.setY(Double.parseDouble(elements[SIZE_Y_IDX]));
      size.setZ(Double.parseDouble(elements[SIZE_Z_IDX]));
      externalObject.setSize(size);

      // Build Pose with Covariance
      geometry_msgs.PoseWithCovariance poseWithCovar = externalObject.getPose();
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

      double[] poseCovariance = new double[COVARINCE_ELEMENT_COUNT];
      for (int i = 0; i < COVARINCE_ELEMENT_COUNT; i++){
        poseCovariance[i] = Double.parseDouble(elements[MIN_POSE_COVAR_IDX + i]);
      }

      poseWithCovar.setCovariance(poseCovariance);
      externalObject.setPose(poseWithCovar);

      // Build Velocity (TwistWithCovariance)
      geometry_msgs.TwistWithCovariance twistWithCovar = externalObject.getVelocity();
      geometry_msgs.Twist twist = twistWithCovar.getTwist();
      geometry_msgs.Vector3 angularVel = twist.getAngular();
      geometry_msgs.Vector3 linearVel = twist.getLinear();
      angularVel.setX(Double.parseDouble(elements[VEL_ANG_X_IDX]));
      angularVel.setY(Double.parseDouble(elements[VEL_ANG_Y_IDX]));
      angularVel.setZ(Double.parseDouble(elements[VEL_ANG_Z_IDX]));

      linearVel.setX(Double.parseDouble(elements[VEL_LIN_X_IDX]));
      linearVel.setY(Double.parseDouble(elements[VEL_LIN_Y_IDX]));
      linearVel.setZ(Double.parseDouble(elements[VEL_LIN_Z_IDX]));

      twist.setAngular(angularVel);
      twist.setLinear(linearVel);
      twistWithCovar.setTwist(twist);

      double[] velocityCovariance = new double[COVARINCE_ELEMENT_COUNT];
      for (int i = 0; i < COVARINCE_ELEMENT_COUNT; i++){
        velocityCovariance[i] = Double.parseDouble(elements[MIN_VEL_COVAR_IDX + i]);
      }

      twistWithCovar.setCovariance(velocityCovariance);

      // Build Velocity Instantaneous (TwistWithCovariance)
      geometry_msgs.TwistWithCovariance twistInstWithCovar = externalObject.getVelocity();
      geometry_msgs.Twist twistInst = twistInstWithCovar.getTwist();
      geometry_msgs.Vector3 angularVelInst = twistInst.getAngular();
      geometry_msgs.Vector3 linearVelInst = twistInst.getLinear();
      angularVelInst.setX(Double.parseDouble(elements[VEL_INST_ANG_X_IDX]));
      angularVelInst.setY(Double.parseDouble(elements[VEL_INST_ANG_Y_IDX]));
      angularVelInst.setZ(Double.parseDouble(elements[VEL_INST_ANG_Z_IDX]));

      linearVelInst.setX(Double.parseDouble(elements[LIN_INST_ANG_X_IDX]));
      linearVelInst.setY(Double.parseDouble(elements[LIN_INST_ANG_Y_IDX]));
      linearVelInst.setZ(Double.parseDouble(elements[LIN_INST_ANG_Z_IDX]));

      twistInst.setAngular(angularVelInst);
      twistInst.setLinear(linearVelInst);
      twistInstWithCovar.setTwist(twistInst);

      double[] velocityInstCovariance = new double[COVARINCE_ELEMENT_COUNT];
      for (int i = 0; i < COVARINCE_ELEMENT_COUNT; i++){
        velocityInstCovariance[i] = Double.parseDouble(elements[MIN_VEL_INST_COVAR_IDX + i]);
      }

      twistInstWithCovar.setCovariance(velocityInstCovariance);
      externalObject.setVelocityInst(twistInstWithCovar);

      // Add external object to list of detected objects
      objects.add(externalObject);
    }

    // Make message
    cav_msgs.ExternalObjectList objectListMsg = flrrObjectPub.newMessage();
    objectListMsg.getHeader().setFrameId(frameId);
    objectListMsg.getHeader().setStamp(currentTime);
    objectListMsg.setObjects(objects);

    // Publish data
    // Only publish actual data on one topic the rest will be empty lists
    flrrObjectPub.publish(objectListMsg);
    // TODO: Un-comment when other radars are used in sensor fusion
    // The frame_id will also need to be updated on each
    // lfsrrObjectPub.publish(lfsrrObjectPub.newMessage());
    // rsrrObjectPub.publish(rsrrObjectPub.newMessage());
    // rfsrrObjectPub.publish(rfsrrObjectPub.newMessage());
    // visionObjectPub.publish(visionObjectPub.newMessage());
  }

  @Override protected short getExpectedColCount() {
    return EXPECTED_DATA_COL_COUNT;
  }

  @Override protected short getSampleIdIdx() {
    return SAMPLE_ID_IDX;
  }

  @Override protected List<String> getDriverTypesList() {
    return new ArrayList<>(Arrays.asList("sensor"));
  }

  @Override public List<String> getDriverAPI(){
    return new ArrayList<>(Arrays.asList(
    "f_lrr/sensor/objects",
    "lf_srr/sensor/objects",
    "r_srr/sensor/objects",
    "rf_srr/sensor/objects",
    "vision/sensor/objects"
    ));
  }
}
