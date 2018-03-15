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

import cav_msgs.MobilityRequest;
import cav_msgs.SystemAlert;

import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;
import org.ros.concurrent.CancellableLoop;
import org.ros.namespace.GraphName;

/**
 * The Negotiator package responsibility is to manage the details of negotiating tactical and strategic
 * agreements between the host vehicle and any other transportation system entities.
 * <p>
 * Command line test: rosrun carma negotiator gov.dot.fhwa.saxton.carma.negotiator.NegotiatorMgr
 * Simple example
 * rostopic pub /system_alert cav_msgs/SystemAlert '{type: 5, description: hello}'
 * rostopic pub /new_plan_outbound cav_msgs/NewPlan '{header: {sender_id: a, plan_id: 44, checksum: 0}}'
 */

    //TODO - this is a minimalist implementation to get us through simplistic, "happy path" lane change maneuvers only.
    //       The structure, data flows, and even message structures, need to be refactored for more general uses.


public class NegotiatorMgr extends SaxtonBaseNode{

  protected ConnectedNode connectedNode;
  protected boolean       systemReady    = false;
  protected SaxtonLogger  log;
  protected int           timeDelay = 5000;
 
  // Topics
  // Publishers
  protected Publisher<cav_msgs.MobilityRequest>      mobReqOutPub;

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
    mobReqOutPub   = connectedNode.newPublisher("/saxton_cav/guidance/outgoing_mobility_request", cav_msgs.MobilityRequest._TYPE);
    timeDelay      = connectedNode.getParameterTree().getInteger("~sleep_duration", 5000);

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
        if (systemReady) {
            //This is a test for Mobility Introduction message
            MobilityRequest requestMsg = mobReqOutPub.newMessage();
            requestMsg.getHeader().setSenderId("DOT-45100");
            requestMsg.getHeader().setRecipientId("");
            requestMsg.getHeader().setSenderBsmId("10ABCDEF");
            requestMsg.getHeader().setPlanId("11111111-2222-3333-AAAA-111111111111");
            requestMsg.getHeader().setTimestamp(System.currentTimeMillis());
            requestMsg.setStrategy("Carma/Platooning");
            requestMsg.getPlanType().setType((byte) 0);
            requestMsg.setUrgency((short) 999);
            requestMsg.getLocation().setEcefX(555555);
            requestMsg.getLocation().setEcefY(666666);
            requestMsg.getLocation().setEcefZ(777777);
            requestMsg.getLocation().setTimestamp(0);
            requestMsg.setStrategyParams("ARG1:5.0, ARG2:16.0");
            mobReqOutPub.publish(requestMsg);
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