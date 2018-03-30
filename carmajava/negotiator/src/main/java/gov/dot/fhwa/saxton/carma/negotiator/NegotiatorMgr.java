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

package gov.dot.fhwa.saxton.carma.negotiator;

import gov.dot.fhwa.saxton.carma.rosutils.AlertSeverity;
import gov.dot.fhwa.saxton.carma.rosutils.SaxtonBaseNode;
import gov.dot.fhwa.saxton.carma.rosutils.SaxtonLogger;

import org.ros.message.MessageListener;
import org.ros.node.topic.Subscriber;

import cav_msgs.LocationOffsetECEF;
import cav_msgs.MobilityOperation;
import cav_msgs.MobilityPath;
import cav_msgs.MobilityRequest;
import cav_msgs.MobilityResponse;
import cav_msgs.SystemAlert;

import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;
import org.ros.concurrent.CancellableLoop;
import org.ros.namespace.GraphName;

/**
 * The Negotiator package responsibility is to manage the details of negotiating tactical and strategic
 * agreements between the host vehicle and any other transportation system entities.
 * TODO This node will be removed in future. Now it is just a test tool for different Mobility messages
 */

public class NegotiatorMgr extends SaxtonBaseNode{

  protected ConnectedNode connectedNode;
  protected boolean       systemReady    = false;
  protected SaxtonLogger  log;
  protected int           timeDelay = 5000;
 
  // Topics
  // Publishers
  protected Publisher<cav_msgs.MobilityRequest>   mobReqOutPub;
  protected Publisher<cav_msgs.MobilityPath>      mobPathOutPub;
  protected Publisher<cav_msgs.MobilityResponse>  mobResOutPub;
  protected Publisher<cav_msgs.MobilityOperation> mobOperPub;

  // Subscribers
  protected Subscriber<cav_msgs.SystemAlert>         alertSub;

  @Override public GraphName getDefaultNodeName() {
    return GraphName.of("negotiator_mgr");
  }

  @Override public void onSaxtonStart(final ConnectedNode connectedNode) {
    this.connectedNode = connectedNode;
    log = new SaxtonLogger(NegotiatorMgr.class.getSimpleName(), connectedNode.getLog());
    // Topics
    // Publishers
    mobReqOutPub  = connectedNode.newPublisher("/saxton_cav/guidance/outgoing_mobility_request", cav_msgs.MobilityRequest._TYPE);
    mobPathOutPub = connectedNode.newPublisher("/saxton_cav/guidance/outgoing_mobility_path", MobilityPath._TYPE);
    mobResOutPub  = connectedNode.newPublisher("/saxton_cav/guidance/outgoing_mobility_response", MobilityResponse._TYPE);
    mobOperPub    = connectedNode.newPublisher("/saxton_cav/guidance/outgoing_mobility_operation", MobilityOperation._TYPE);
    timeDelay     = connectedNode.getParameterTree().getInteger("~sleep_duration", 5000);

    alertSub = connectedNode.newSubscriber("system_alert", cav_msgs.SystemAlert._TYPE);
    alertSub.addMessageListener(new MessageListener<cav_msgs.SystemAlert>() {
      @Override public void onNewMessage(cav_msgs.SystemAlert message) {
          try {
            handleSystemAlertMsg(message);
          } catch (Exception e) {
            handleException(e);
          }//try

      }//onNewMessage
    });//addMessageListener
    
    connectedNode.executeCancellableLoop(new CancellableLoop() {
      @Override protected void loop() throws InterruptedException {
        if (systemReady && false) {
            //This is a test for Mobility Introduction message
            MobilityRequest requestMsg = mobReqOutPub.newMessage();
            requestMsg.getHeader().setSenderId("DOT-45100");
            requestMsg.getHeader().setRecipientId("");
            requestMsg.getHeader().setSenderBsmId("10ABCDEF");
            requestMsg.getHeader().setPlanId("11111111-2222-3333-AAAA-111111111111");
            requestMsg.getHeader().setTimestamp(System.currentTimeMillis());
            requestMsg.setStrategy("FakeStrategy");
            requestMsg.getPlanType().setType((byte) 0);
            requestMsg.setUrgency((short) 999);
            requestMsg.getLocation().setEcefX(555555);
            requestMsg.getLocation().setEcefY(666666);
            requestMsg.getLocation().setEcefZ(777777);
            requestMsg.getLocation().setTimestamp(0);
            requestMsg.setStrategyParams("ARG1:5.0, ARG2:16.0");
            mobReqOutPub.publish(requestMsg);

            MobilityPath pathMsg = mobPathOutPub.newMessage();
            pathMsg.getHeader().setSenderId("DOT-45100");
            pathMsg.getHeader().setRecipientId("");
            pathMsg.getHeader().setSenderBsmId("10ABCDEF");
            pathMsg.getHeader().setPlanId("11111111-2222-3333-BBBB-111111111111");
            pathMsg.getHeader().setTimestamp(System.currentTimeMillis());
            pathMsg.getTrajectory().getLocation().setEcefX(5555);
            pathMsg.getTrajectory().getLocation().setEcefY(6666);
            pathMsg.getTrajectory().getLocation().setEcefZ(7777);
            pathMsg.getTrajectory().getLocation().setTimestamp(0);
            LocationOffsetECEF offset1 = connectedNode.getTopicMessageFactory().newFromType(LocationOffsetECEF._TYPE);
            offset1.setOffsetX((short) 1);
            offset1.setOffsetY((short) 2);
            offset1.setOffsetZ((short) 3);
            LocationOffsetECEF offset2 = connectedNode.getTopicMessageFactory().newFromType(LocationOffsetECEF._TYPE);
            offset2.setOffsetX((short) 4);
            offset2.setOffsetY((short) 5);
            offset2.setOffsetZ((short) 6);
            LocationOffsetECEF offset3 = connectedNode.getTopicMessageFactory().newFromType(LocationOffsetECEF._TYPE);
            offset3.setOffsetX((short) 7);
            offset3.setOffsetY((short) 8);
            offset3.setOffsetZ((short) 9);
            pathMsg.getTrajectory().getOffsets().add(offset1);
            pathMsg.getTrajectory().getOffsets().add(offset2);
            pathMsg.getTrajectory().getOffsets().add(offset3);
            mobPathOutPub.publish(pathMsg);
            
            MobilityResponse response = mobResOutPub.newMessage();
            response.getHeader().setSenderId("DOT-45100");
            response.getHeader().setRecipientId("");
            response.getHeader().setSenderBsmId("10ABCDEF");
            response.getHeader().setPlanId("11111111-2222-3333-AAAA-111111111111");
            response.getHeader().setTimestamp(System.currentTimeMillis());
            response.setIsAccepted(true);
            response.setUrgency((short) 500);
            mobResOutPub.publish(response);
            
            MobilityOperation op = mobOperPub.newMessage();
            op.getHeader().setSenderId("DOT-45100");
            op.getHeader().setRecipientId("");
            op.getHeader().setSenderBsmId("10ABCDEF");
            op.getHeader().setPlanId("11111111-2222-3333-AAAA-111111111111");
            op.getHeader().setTimestamp(System.currentTimeMillis());
            op.setStrategy("FakeStrategy");
            op.setStrategyParams("STATUS|CMDSPEED:10.01,DTD:50.02,SPEED:10.01");
            mobOperPub.publish(op);
        }
        Thread.sleep(timeDelay);
      }
    });
  }//onStart

  /***
   * Handles unhandled exceptions and reports to SystemAlert topic, and log the alert.
   * @param e The exception to handle
   */
  @Override
  protected void handleException(Throwable e) {

    String msg = "Uncaught exception in " + connectedNode.getName() + " caught by handleException";
    publishSystemAlert(AlertSeverity.FATAL, msg, e);
    connectedNode.shutdown();
  }

  /*
  	Basic shutdown procedure.
  	Add more procedures here as needed.
   */
  public void shutdown() {
    log.info("SHUTDOWN", "Negotiator shutdown method called.");
    this.connectedNode.shutdown();
  }

  /**
   * Function to be used as a callback for received system alert messages
   *
   * @param msg the system alert message
   *
   * TODO: Create additional methods to handle the different alerts.
   */
  protected void handleSystemAlertMsg(SystemAlert msg) {
    switch (msg.getType()) {
      case cav_msgs.SystemAlert.CAUTION:
        // TODO: Handle this message type
        break;
      case cav_msgs.SystemAlert.WARNING:
        // TODO: Handle this message type
        break;
      case cav_msgs.SystemAlert.FATAL:
        //TODO:  Handle this message type
        log.info("SHUTDOWN", "Negotiator received system fatal on system_alert and will be shutting down");
        shutdown();
        break;
      case cav_msgs.SystemAlert.NOT_READY:
        //TODO:  Handle this message type
        log.info("STARTUP", "Negotiator received system not ready on system_alert.");
        break;
      case cav_msgs.SystemAlert.DRIVERS_READY:
        //TODO:  Handle this message type
        systemReady = true;
        log.info("STARTUP", "Negotiator received drivers ready on system_alert.");
        break;
      case cav_msgs.SystemAlert.SHUTDOWN:
        log.info("SHUTDOWN", "Negotiator received a shutdown message");
        shutdown();
        break;
      default:
        //TODO: Handle this variant maybe throw exception?
        log.error("ALERT", "Negotiator received a system alert message with unknown type: " + msg.getType());
    }
  }
}//AbstractNodeMain
