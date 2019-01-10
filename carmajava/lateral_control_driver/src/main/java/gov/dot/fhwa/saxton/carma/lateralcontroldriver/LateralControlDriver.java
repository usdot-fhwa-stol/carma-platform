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

package gov.dot.fhwa.saxton.carma.lateralcontroldriver;

import gov.dot.fhwa.saxton.carma.rosutils.SaxtonBaseNode;
import org.ros.concurrent.CancellableLoop;
import org.ros.node.ConnectedNode;
import org.ros.node.parameter.ParameterTree;
import org.ros.namespace.GraphName;
import cav_srvs.*;
import gov.dot.fhwa.saxton.carma.rosutils.AlertSeverity;
import gov.dot.fhwa.saxton.carma.rosutils.SaxtonLogger;
import org.ros.message.Time;
import org.ros.node.topic.Subscriber;
import org.ros.node.topic.Publisher;
import org.ros.node.service.ServiceServer;
import java.util.LinkedList;
import java.util.Arrays;
import java.util.List;

/**
 * Node which functions as a lateral control driver for the carma platform
 * <p>
 * Command line test:
 * rosrun carmajava lateral_control_driver gov.dot.fhwa.saxton.carma.lateralcontroldriver.LateralControlDriver
 */
public class LateralControlDriver extends SaxtonBaseNode implements ILateralControlDriver {

  private LateralControlWorker worker_;

  // Topics
  // Publishers
  private Publisher<cav_msgs.UIInstructions> uiInstructionsPub_;
  private Publisher<cav_msgs.DriverStatus> discoveryPub_;
  // Subscribers
  private Subscriber<cav_msgs.LateralControl> lateralControlSub_;

  // Services
  // Provided
  protected ServiceServer<BindRequest, BindResponse> bindService;
  protected ServiceServer<GetDriverApiRequest, GetDriverApiResponse> getApiService;
  protected ServiceServer<GetDriverStatusRequest, GetDriverStatusResponse> getStatusService;

  private String uiInstuctionsTopic = "ui_instructions";
  private String lateralControlTopic = "~control/cmd_lateral";

  private SaxtonLogger log_;
  private ParameterTree params_;

  private ConnectedNode connectedNode_;

  @Override public GraphName getDefaultNodeName() {
    return GraphName.of("lateral_controller");
  }

  @Override public void onSaxtonStart(final ConnectedNode connectedNode) {

    connectedNode_ = connectedNode;
    log_ = new SaxtonLogger(this.getClass().getSimpleName(), connectedNode.getLog());
    params_ = connectedNode.getParameterTree();
    double angleThreshold = params_.getDouble("~angle_threshold", 0.0349066); // 2 deg

    // Topics
    // Publishers
    uiInstructionsPub_ = connectedNode.newPublisher(uiInstuctionsTopic, cav_msgs.UIInstructions._TYPE);
    //bondPub = connectedNode.newPublisher("~/bond", bond.Status._TYPE); //TODO add once bind cpp is wrapped in jni
    discoveryPub_ = connectedNode.newPublisher("driver_discovery", cav_msgs.DriverStatus._TYPE);

    // Worker must be initialized after publishers but before subscribers
    worker_ = new LateralControlWorker(this, log_, angleThreshold);

    // Subscribers
    lateralControlSub_ = connectedNode.newSubscriber(lateralControlTopic, cav_msgs.LateralControl._TYPE);
    lateralControlSub_.addMessageListener(
      (cav_msgs.LateralControl msg) -> {
        try {
          worker_.handleLateralControlMsg(msg);
        } catch (Throwable e) {
          handleException(e);
        }
      });

    // Service
    // Server
    bindService = connectedNode.newServiceServer("~/bind", cav_srvs.Bind._TYPE,
      (BindRequest req, BindResponse res) -> {
        log_.info("Request for bind received");
      });
        
    getApiService = connectedNode.newServiceServer("~/get_driver_api", cav_srvs.GetDriverApi._TYPE,
      (GetDriverApiRequest req, GetDriverApiResponse res) -> {
        res.setApiList(getDriverApi());
      });

    getStatusService = connectedNode.newServiceServer("~/get_status", GetDriverStatus._TYPE,
      (GetDriverStatusRequest req, GetDriverStatusResponse res) -> {
        cav_msgs.DriverStatus msg = discoveryPub_.newMessage();
        msg.setName(connectedNode.getName().toString());
        msg.setStatus(worker_.getDriverStatus());
        msg.setLatController(true);
        res.setStatus(msg);
      });

    // This CancellableLoop will be canceled automatically when the node shuts down.
    connectedNode.executeCancellableLoop(new CancellableLoop() {
      @Override
      protected void loop() throws InterruptedException {
        cav_msgs.DriverStatus msg = discoveryPub_.newMessage();
        msg.setName(connectedNode.getName().toString());
        msg.setStatus(worker_.getDriverStatus());
        msg.setLatController(true);
        discoveryPub_.publish(msg); // Publish driver discovery message
        Thread.sleep(1000);
      }//loop
    });//executeCancellableLoop
    
  }//onStart

  /**
   * Function returns the list of topics which form this driver's api
   * @return a list of fully qualified topic names
   */
  public List<String> getDriverApi() {
    return new LinkedList<String>(Arrays.asList(lateralControlSub_.getTopicName().toString()));
  }

  @Override public void publishUIMessage(cav_msgs.UIInstructions msg) {
    uiInstructionsPub_.publish(msg);
  }

  @Override protected void handleException(Throwable e) {
    String msg = "Uncaught exception in " + connectedNode_.getName() + " caught by handleException";
    publishSystemAlert(AlertSeverity.FATAL, msg, e);
    connectedNode_.shutdown();
  }

  @Override public Time getTime() {
    return connectedNode_.getCurrentTime();
  }
}//SaxtonBaseNode
