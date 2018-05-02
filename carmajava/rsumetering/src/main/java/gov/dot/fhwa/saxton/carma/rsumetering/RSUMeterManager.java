/*
 * Copyright (C) 2018 LEIDOS.
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

package gov.dot.fhwa.saxton.carma.rsumetering;

import gov.dot.fhwa.saxton.carma.rosutils.SaxtonBaseNode;
import org.ros.concurrent.CancellableLoop;
import org.ros.node.ConnectedNode;
import org.ros.node.parameter.ParameterTree;
import org.ros.namespace.GraphName;
import gov.dot.fhwa.saxton.carma.rosutils.AlertSeverity;
import gov.dot.fhwa.saxton.carma.rosutils.SaxtonLogger;
import org.ros.message.Time;
import org.ros.node.topic.Subscriber;
import org.ros.node.topic.Publisher;
import cav_msgs.MobilityRequest;
import cav_msgs.MobilityOperation;

/**
 * Node which functions as a messaging logic controller for an RSU based ramp metering system
 * <p>
 * Command line test:
 * rosrun carma rsumetering gov.dot.fhwa.saxton.carma.rsumetering.RSUMeterManager
 */
public class RSUMeterManager extends SaxtonBaseNode implements IRSUMeterManager {

  private RSUMeterWorker worker;

  // Topics
  // Publishers
  private Publisher<MobilityRequest> requestPub;
  private Publisher<MobilityOperation> operationPub;
  // Subscribers
  private Subscriber<MobilityOperation> operationSub;

  private String outgoingRequestTopic = "outgoing_mobility_request";
  private String outgoingOperationTopic = "outgoing_mobility_operation";
  private String incomingOperationTopic = "incoming_mobility_operation";

  private SaxtonLogger log;
  private ParameterTree params;

  private ConnectedNode connectedNode;

  @Override public GraphName getDefaultNodeName() {
    return GraphName.of("rsu_meter");
  }

  @Override public void onSaxtonStart(final ConnectedNode connectedNode) {

    this.connectedNode = connectedNode;
    this.log = new SaxtonLogger(this.getClass().getSimpleName(), connectedNode.getLog());
    this.params = connectedNode.getParameterTree();
    String routeFilePath = params.getString("~route_file");
    double distToMergOnRamp = params.getDouble("~dist_to_merge_on_ramp");
    String targetId = params.getString("~target_id");
    double mergeDTD = params.getDouble("~merge_downtrack_distance");

    log.info("LoadedParam route_file: " + routeFilePath);
    log.info("LoadedParam dist_to_merge: " + distToMergOnRamp);
    log.info("LoadedParam target_id: " + targetId);
    log.info("LoadedParam merge_downtrack_distance: " + mergeDTD);


    // Topics
    // Publishers
    requestPub = connectedNode.newPublisher(outgoingRequestTopic, MobilityRequest._TYPE);
    operationPub = connectedNode.newPublisher(outgoingOperationTopic, MobilityOperation._TYPE);

    // Worker must be initialized after publishers but before subscribers
    worker = new RSUMeterWorker(this, log, routeFilePath, distToMerge, targetId, mergeWPId);

    // Subscribers
    operationSub = connectedNode.newSubscriber(incomingOperationTopic, MobilityOperation._TYPE);
    operationSub.addMessageListener(
      (MobilityOperation msg) -> {
        try {
          worker.handleMobilityOperationMsg(msg);
        } catch (Throwable e) {
          handleException(e);
        }
      });

    // This CancellableLoop will be canceled automatically when the node shuts down.
    connectedNode.executeCancellableLoop(new CancellableLoop() {
      @Override
      protected void loop() throws InterruptedException {
        Thread.sleep(1000);
      }//loop
    });//executeCancellableLoop
    
  }//onStart

  @Override public void publishMobilityRequest(MobilityRequest msg) {
    requestPub.publish(msg);
  }

  @Override protected void handleException(Throwable e) {
    String msg = "Uncaught exception in " + connectedNode.getName() + " caught by handleException";
    publishSystemAlert(AlertSeverity.FATAL, msg, e);
    connectedNode.shutdown();
  }

  @Override public Time getTime() {
    return connectedNode.getCurrentTime();
  }

  @Override
  public void publishMobilityOperation(MobilityOperation msg) {
    operationPub.publish(msg);
  }
}//SaxtonBaseNode
