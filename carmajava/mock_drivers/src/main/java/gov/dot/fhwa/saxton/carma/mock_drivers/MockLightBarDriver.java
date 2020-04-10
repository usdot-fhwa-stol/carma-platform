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
import org.ros.exception.ServiceException;
import org.ros.node.service.ServiceResponseBuilder;
import org.ros.node.service.ServiceServer;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import cav_msgs.LightBarStatus;
import cav_srvs.GetLightsRequest;
import cav_srvs.GetLightsResponse;
import cav_srvs.SetLightsRequest;
import cav_srvs.SetLightsResponse;


/**
 * A class which can be used to simulate a LightBar driver for the CARMA Platform.
 * <p>
 * Command line test:
 * ROSJava does not support rosrun parameter setting so a rosrun is a multi step process
 * rosparam set /mock_driver/simulated_driver 'pinpoint'
 * rosparam set /mock_driver/data_file_path '/opt/carma/test_data/pinpoint_stationary.csv'
 * rosrun carma mock_drivers gov.dot.fhwa.saxton.carma.mock_drivers.MockDriverNode
 */
public class MockLightBarDriver extends AbstractMockDriver {

  // Topics
  // Published
  final Publisher<cav_msgs.LightBarStatus> lbPub;
  final String lbStatusTopic = "lightbar/light_bar_status";

  // Services
  // Server
  protected final ServiceServer<cav_srvs.SetLightsRequest, cav_srvs.SetLightsResponse> setLightsService;
  protected final ServiceServer<cav_srvs.GetLightsRequest, cav_srvs.GetLightsResponse> getLightsService;
  final String lbGetLightsService = "lightbar/get_lights";
  final String lbSetLightsService = "lightbar/set_lights";  

  private final short EXPECTED_DATA_COL_COUNT = 132; // TODO
  private final short SAMPLE_ID_IDX = 1;

  // LightBar States
  protected boolean greenFlash = false;
  protected boolean yellowFlash = false;
  protected boolean leftArrow = false;
  protected boolean rightArrow = false;
  protected boolean sidesSolid = false;
  protected boolean greenSolid = false;
  protected boolean yellowDim = false;
  protected boolean takedown = false;


  public MockLightBarDriver (ConnectedNode connectedNode) {
    super(connectedNode);
    // Topics
    // Published
    lbPub =
      connectedNode.newPublisher(lbStatusTopic, cav_msgs.LightBarStatus._TYPE);

    // Services
    // Server
    setLightsService = connectedNode.newServiceServer(lbSetLightsService, cav_srvs.SetLights._TYPE,
        new ServiceResponseBuilder<SetLightsRequest, SetLightsResponse>() {
          @Override
          public void build(cav_srvs.SetLightsRequest request, cav_srvs.SetLightsResponse response) {

            cav_msgs.LightBarStatus lightStatus = request.getSetState();
            greenFlash = lightStatus.getGreenFlash() == 1;
            yellowFlash = lightStatus.getFlash() == 1;
            leftArrow = lightStatus.getLeftArrow() == 1;
            rightArrow = lightStatus.getRightArrow() == 1;
            sidesSolid = lightStatus.getSidesSolid() == 1;
            greenSolid = lightStatus.getGreenSolid() == 1;
            yellowDim = lightStatus.getYellowSolid() == 1;
            takedown = lightStatus.getTakedown() == 1;
  
            log.info("Lights have been set to: " + 
            "\nGreen Flash: " + greenFlash + 
            "\nYellow Flash: " + yellowFlash + 
            "\nLeft: " + leftArrow + 
            "\nRight: " + rightArrow + 
            "\nSides Solid: " + sidesSolid + 
            "\nGreen Solid: " + greenSolid + 
            "\nYellow Dim: " + yellowDim + 
            "\nTakedown: " + takedown);
          }
        });

    getLightsService = connectedNode.newServiceServer(lbGetLightsService, cav_srvs.GetLights._TYPE,
        new ServiceResponseBuilder<GetLightsRequest, GetLightsResponse>() {
          @Override
          public void build(cav_srvs.GetLightsRequest request, cav_srvs.GetLightsResponse response) {
          
          response.setStatus(getLightBarStatus());

          }
        });
  }

  /**
   * Helper function to build the lightbar status message
   * @return The lightbar status message
   */
  protected LightBarStatus getLightBarStatus() {
    cav_msgs.LightBarStatus lightStatus = lbPub.newMessage();

    lightStatus.setGreenSolid((byte)(greenSolid?1:0));
    lightStatus.setYellowSolid((byte)(yellowDim?1:0));
    lightStatus.setRightArrow((byte)(rightArrow?1:0));
    lightStatus.setLeftArrow((byte)(leftArrow?1:0));
    lightStatus.setSidesSolid((byte)(sidesSolid?1:0));
    lightStatus.setFlash((byte)(yellowFlash?1:0));
    lightStatus.setGreenFlash((byte)(greenFlash?1:0));
    lightStatus.setTakedown((byte)(takedown?1:0));

    return lightStatus;
  }
  
  @Override protected void publishData(List<String[]> data) throws IllegalArgumentException {
    // TODO use actual data from file
    cav_msgs.LightBarStatus lightStatus = getLightBarStatus();
  
    lbPub.publish(lightStatus);
  }

  @Override protected short getExpectedColCount() {
    return EXPECTED_DATA_COL_COUNT;
  }

  @Override protected short getSampleIdIdx(){
    return SAMPLE_ID_IDX;
  }

  @Override protected List<String> getDriverTypesList() {
    return new ArrayList<>(Arrays.asList("lightbar"));
  }

  @Override public List<String> getDriverAPI() {
    return new ArrayList<>(Arrays.asList(
      lbStatusTopic,
      lbGetLightsService,
      lbSetLightsService));
  }
}
