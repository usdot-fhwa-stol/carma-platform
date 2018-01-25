/*
 * Copyright (C) 2017 LEIDOS.
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

import cav_msgs.*;
import gov.dot.fhwa.saxton.carma.rosutils.AlertSeverity;
import gov.dot.fhwa.saxton.carma.rosutils.SaxtonBaseNode;
import gov.dot.fhwa.saxton.carma.rosutils.SaxtonLogger;

import java.nio.ByteOrder;
import java.util.Arrays;

import org.jboss.netty.buffer.ChannelBuffers;
import org.ros.message.MessageListener;
import org.ros.node.topic.Subscriber;
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
  protected MobilityPlan  lastNewPlanMsg = null;
  protected SaxtonLogger  log;
  protected int           timeDelay = 5000;
 
  // Topics
  // Publishers
  protected Publisher<cav_msgs.MobilityAck>      mobAckOutPub;
  protected Publisher<cav_msgs.MobilityGreeting> mobGreetOutPub;
  protected Publisher<cav_msgs.MobilityIntro>    mobIntroOutPub;
  protected Publisher<cav_msgs.MobilityPlan>     mobPlanOutPub;

  // Subscribers
  protected Subscriber<cav_msgs.MobilityAck>      mobAckInSub;
  protected Subscriber<cav_msgs.MobilityGreeting> mobGreetInSub;
  protected Subscriber<cav_msgs.MobilityIntro>    mobIntroInSub;
  protected Subscriber<cav_msgs.MobilityPlan>     mobPlanInSub;
  protected Subscriber<cav_msgs.SystemAlert>      alertSub;

  @Override public GraphName getDefaultNodeName() {
    return GraphName.of("negotiator_mgr");
  }

  @Override public void onSaxtonStart(final ConnectedNode connectedNode) {
    this.connectedNode = connectedNode;
    log = new SaxtonLogger(NegotiatorMgr.class.getSimpleName(), connectedNode.getLog());
    // Topics
    // Publishers
    mobAckOutPub   = connectedNode.newPublisher("mobility_ack_outbound",     cav_msgs.MobilityAck._TYPE);
    mobGreetOutPub = connectedNode.newPublisher("mobility_greeting_outbound",cav_msgs.MobilityGreeting._TYPE);
    mobIntroOutPub = connectedNode.newPublisher("mobility_intro_outbound",   cav_msgs.MobilityIntro._TYPE);
    mobPlanOutPub  = connectedNode.newPublisher("mobility_plan_outbound",    cav_msgs.MobilityPlan._TYPE);
    timeDelay  = connectedNode.getParameterTree().getInteger("~sleep_duration", 5000);

    mobAckInSub = connectedNode.newSubscriber("mobility_ack_inbound", cav_msgs.MobilityAck._TYPE);
    mobAckInSub.addMessageListener(new MessageListener<cav_msgs.MobilityAck>() {
      @Override public void onNewMessage(cav_msgs.MobilityAck message) {
        log.info("V2V", "Negotiator received new MobilityAck");
      }//onNewMessage
    });//addMessageListener

    mobGreetInSub = connectedNode.newSubscriber("mobility_greeting_inbound", cav_msgs.MobilityGreeting._TYPE);
    mobGreetInSub.addMessageListener(new MessageListener<cav_msgs.MobilityGreeting>() {
      @Override public void onNewMessage(cav_msgs.MobilityGreeting message) {
        log.info("V2V", "Negotiator received new MobilityGreeting");
      }//onNewMessage
    });//addMessageListener

    mobIntroInSub = connectedNode.newSubscriber("mobility_intro_inbound", cav_msgs.MobilityIntro._TYPE);
    mobIntroInSub.addMessageListener(new MessageListener<cav_msgs.MobilityIntro>() {
      @Override public void onNewMessage(cav_msgs.MobilityIntro message) {
        log.info("V2V", "Negotiator received new MobilityIntro");
      }//onNewMessage
    });//addMessageListener

    mobPlanInSub = connectedNode.newSubscriber("mobility_plan_inbound", cav_msgs.MobilityPlan._TYPE);
    mobPlanInSub.addMessageListener(new MessageListener<cav_msgs.MobilityPlan>() {
      @Override public void onNewMessage(cav_msgs.MobilityPlan message) {
        log.info("V2V", "Negotiator received new MobilityPlan");
      }//onNewMessage
    });//addMessageListener

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
      @Override protected void setup() {

      }
      @Override protected void loop() throws InterruptedException {
        if (systemReady) {
            //This is a test for Mobility Introduction message
            MobilityIntro introMsg = mobIntroOutPub.newMessage();
            byte[] hostId = new byte[16];
            Arrays.fill(hostId, (byte) 1);
            introMsg.getHeader().setSenderId(ChannelBuffers.copiedBuffer(ByteOrder.LITTLE_ENDIAN, hostId));
            byte[] targetId = new byte[16];
            Arrays.fill(targetId, (byte) 0);
            introMsg.getHeader().setRecipientId(ChannelBuffers.copiedBuffer(ByteOrder.LITTLE_ENDIAN, targetId));
            byte[] planId = new byte[16];
            Arrays.fill(planId, (byte) 2);
            introMsg.getHeader().setPlanId(ChannelBuffers.copiedBuffer(ByteOrder.LITTLE_ENDIAN, planId));
            introMsg.getHeader().getTimestamp().setYear((short) 2010);
            introMsg.getHeader().getTimestamp().setMonth((byte) 2);
            introMsg.getHeader().getTimestamp().setDay((byte) 1);
            introMsg.getHeader().getTimestamp().setHour((byte) 0);
            introMsg.getHeader().getTimestamp().setMinute((byte) 0);
            introMsg.getHeader().getTimestamp().setSecond(0);
            introMsg.getMyEntityType().setType((BasicVehicleClass.DEFAULT_PASSENGER_VEHICLE));
            introMsg.setMyRoadwayLink("[Test Road]");
            introMsg.setMyRoadwayLinkPosition((short) 2);
            introMsg.setMyLaneId((byte) 1);
            introMsg.setForwardSpeed((float) 0.2);
            introMsg.getPlanType().setType(PlanType.UNKNOWN);
            introMsg.setProposalParam((short) 100);
            byte[] publicKey = new byte[64];
            Arrays.fill(publicKey, (byte) 0);
            introMsg.setMyPublicKey(ChannelBuffers.copiedBuffer(ByteOrder.LITTLE_ENDIAN, publicKey));
            introMsg.getExpiration().setYear((short) 2018);
            introMsg.getExpiration().setMonth((byte) 6);
            introMsg.getExpiration().setDay((byte) 1);
            introMsg.getExpiration().setHour((byte) 0);
            introMsg.getExpiration().setMinute((byte) 0);
            introMsg.getExpiration().setSecond(0);
            introMsg.getExpiration().setOffset((short) -300);
            introMsg.setCapabilities("[CarmaPlatform v2.2.3]");
            mobIntroOutPub.publish(introMsg);
            MobilityAck ackMsg = mobAckOutPub.newMessage();
            ackMsg.getHeader().setSenderId(ChannelBuffers.copiedBuffer(ByteOrder.LITTLE_ENDIAN, hostId));
            ackMsg.getHeader().setRecipientId(ChannelBuffers.copiedBuffer(ByteOrder.LITTLE_ENDIAN, targetId));
            ackMsg.getHeader().setPlanId(ChannelBuffers.copiedBuffer(ByteOrder.LITTLE_ENDIAN, planId));
            ackMsg.getHeader().getTimestamp().setYear((short) 2018);
            ackMsg.getHeader().getTimestamp().setMonth((byte) 1);
            ackMsg.getHeader().getTimestamp().setDay((byte) 1);
            ackMsg.getHeader().getTimestamp().setHour((byte) 0);
            ackMsg.getHeader().getTimestamp().setMinute((byte) 0);
            ackMsg.getHeader().getTimestamp().setSecond(0);
            ackMsg.getHeader().getTimestamp().setOffset((short) -300);
            ackMsg.getAgreement().setType(MobilityAckType.ACCEPT_WITH_EXECUTE);
            ackMsg.setVerificationCode("");
            mobAckOutPub.publish(ackMsg);
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