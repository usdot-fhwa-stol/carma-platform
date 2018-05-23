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

import gov.dot.fhwa.saxton.carma.geometry.geodesic.Location;
import gov.dot.fhwa.saxton.carma.rosutils.AlertSeverity;
import gov.dot.fhwa.saxton.carma.rosutils.SaxtonLogger;
import org.ros.message.Time;
import org.ros.node.topic.Subscriber;
import org.ros.node.topic.Publisher;
import cav_msgs.MobilityRequest;
import cav_msgs.MobilityResponse;
import cav_msgs.BSM;
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
  private Publisher<MobilityResponse> responsePub;
  // Subscribers
  private Subscriber<MobilityRequest> requestSub;
  private Subscriber<MobilityOperation> operationSub;
  private Subscriber<MobilityResponse> responseSub;
  private Subscriber<BSM> bsmSub;

  private final String outgoingRequestTopic = "outgoing_mobility_request";
  private final String outgoingOperationTopic = "outgoing_mobility_operation";
  private final String outgoingResponseTopic = "outgoing_mobility_response";

  private final String incomingRequestTopic = "incoming_mobility_request";
  private final String incomingOperationTopic = "incoming_mobility_operation";
  private final String incomingResponseTopic = "incoming_mobility_response";
  private final String incomingBSMTopic = "incoming_bsm";

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
    // Load Params
    String routeFilePath = params.getString("~route_file");
    double distToMerg = params.getDouble("~dist_to_merge_along_ramp");
    String rsuId = params.getString("~rsu_id");
    double mainRouteMergeDTD = params.getDouble("~dist_to_merge_on_main_route");
    double lengthOfMerge = params.getDouble("~length_of_merge");
    double rampMeterRadius = params.getDouble("~ramp_meter_radius");
    int targetLane = params.getInteger("~target_lane");
    long timeMargin = params.getInteger("~arrival_time_margin");
    double requestFreq = params.getDouble("~standby_state_request_freq");
    double commandFreq = params.getDouble("~command_freq");
    long requestPeriod = (long) (1000.0 / requestFreq);
    long commandPeriod = (long) (1000.0 / commandFreq);
    long commsTimeout = params.getInteger("~comms_timeout");
    double meterLat = params.getDouble("~meter_point_latitude");
    double meterLon = params.getDouble("~meter_point_longitude");
    double meterAlt = params.getDouble("~meter_point_elevation");
    Location meterLocation = new Location(meterLat, meterLon, meterAlt);
    double minApproachAccel = params.getDouble("~min_approach_accel");
    double targetSpeedBeforeStop = params.getDouble("~target_speed_before_stop");
    double driverLagTime = params.getDouble("~driver_lag_time");
    double commsLagTime = params.getDouble("~comms_lag_time");

    // Echo Params
    log.info("LoadedParam route_file: " + routeFilePath);
    log.info("LoadedParam dist_to_merge_along_ramp: " + distToMerg);
    log.info("LoadedParam rsu_id: " + rsuId);
    log.info("LoadedParam dist_to_merge_on_main_route: " + mainRouteMergeDTD);
    log.info("LoadedParam length_of_merge: " + lengthOfMerge);
    log.info("LoadedParam ramp_meter_radius: " + rampMeterRadius);
    log.info("LoadedParam target_lane: " + targetLane);
    log.info("LoadedParam arrival_time_margin: " + timeMargin);
    log.info("LoadedParam standby_state_request_freq: " + requestFreq);
    log.info("LoadedParam command_freq: " + commandFreq);
    log.info("LoadedParam comms_timeout: " + commsTimeout);
    log.info("LoadedParam meter_point_latitude: " + meterLat);
    log.info("LoadedParam meter_point_longitude: " + meterLon);
    log.info("LoadedParam meter_point_elevation: " + meterAlt);
    log.info("LoadedParam min_approach_accel: " + minApproachAccel);
    log.info("LoadedParam target_speed_before_stop: " + targetSpeedBeforeStop);
    log.info("LoadedParam driver_lag_time: " + driverLagTime);
    log.info("LoadedParam comms_lag_time: " + commsLagTime);

    // Topics
    // Publishers
    requestPub = connectedNode.newPublisher(outgoingRequestTopic, MobilityRequest._TYPE);
    operationPub = connectedNode.newPublisher(outgoingOperationTopic, MobilityOperation._TYPE);
    responsePub = connectedNode.newPublisher(outgoingResponseTopic, MobilityResponse._TYPE);

    // Worker must be initialized after publishers but before subscribers
    worker = new RSUMeterWorker(this, log, routeFilePath, rsuId, distToMerg,
     mainRouteMergeDTD, rampMeterRadius, targetLane, lengthOfMerge, timeMargin,
     requestPeriod, commandPeriod, commsTimeout, meterLocation,
     minApproachAccel, targetSpeedBeforeStop, driverLagTime, commsLagTime
    );

    // Subscribers
    requestSub = connectedNode.newSubscriber(incomingRequestTopic, MobilityRequest._TYPE);
    requestSub.addMessageListener(
      (MobilityRequest msg) -> {
        try {
          worker.handleMobilityRequestMsg(msg);
        } catch (Throwable e) {
          handleException(e);
        }
      });

    operationSub = connectedNode.newSubscriber(incomingOperationTopic, MobilityOperation._TYPE);
    operationSub.addMessageListener(
      (MobilityOperation msg) -> {
        try {
          worker.handleMobilityOperationMsg(msg);
        } catch (Throwable e) {
          handleException(e);
        }
      });

    responseSub = connectedNode.newSubscriber(incomingResponseTopic, MobilityResponse._TYPE);
    responseSub.addMessageListener(
      (MobilityResponse msg) -> {
        try {
          worker.handleMobilityResponseMsg(msg);
        } catch (Throwable e) {
          handleException(e);
        }
      });

    bsmSub = connectedNode.newSubscriber(incomingBSMTopic, BSM._TYPE);
    bsmSub.addMessageListener(
      (BSM msg) -> {
        try {
          worker.handleBSMMsg(msg);
        } catch (Throwable e) {
          handleException(e);
        }
      });

    // This CancellableLoop will be canceled automatically when the node shuts down.
    connectedNode.executeCancellableLoop(new CancellableLoop() {
      @Override
      protected void loop() throws InterruptedException {
        worker.loop();
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

  @Override
  public void publishMobilityResponse(MobilityResponse msg) {
    responsePub.publish(msg);
  }
}//SaxtonBaseNode
