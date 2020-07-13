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
  Publisher<radar_msgs.RadarTrackArray> tracksPub; 
  Publisher<radar_msgs.RadarStatus> statusPub; 

  // TODO 
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
    tracksPub = connectedNode.newPublisher("radar/tracks_raw", radar_msgs.RadarTrackArray._TYPE);
    statusPub = connectedNode.newPublisher("radar/status", radar_msgs.RadarStatus._TYPE);
  }

  @Override protected void publishData(List<String[]> data) {

    String frameId = "f_lrr_frame";
    Time currentTime = connectedNode.getCurrentTime();
    radar_msgs.RadarTrackArray trackMsg = tracksPub.newMessage();
    radar_msgs.RadarStatus statusMsg = statusPub.newMessage();

    // Build Header
    std_msgs.Header hdr = trackMsg.getHeader();
    hdr.setFrameId(frameId);
    hdr.setStamp(currentTime);

    statusMsg.setHeader(hdr);

    tracksPub.publish(trackMsg);
    statusPub.publish(statusMsg);
  }

  @Override protected short getExpectedColCount() {
    return EXPECTED_DATA_COL_COUNT;
  }

  @Override protected short getSampleIdIdx() {
    return SAMPLE_ID_IDX;
  }

  @Override protected List<String> getDriverTypesList() {
    return new ArrayList<>(Arrays.asList("radar"));
  }

  @Override public List<String> getDriverAPI(){
    return new ArrayList<>(Arrays.asList(
    "radar/tracks_raw",
    "radar/status"
    ));
  }
}
