/*
 * TODO: Copyright (C) 2017 LEIDOS.
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

//Originally "com.github.rosjava.carma.template;"
package gov.dot.fhwa.saxton.carma.message;

import cav_msgs.*;
import gov.dot.fhwa.saxton.carma.rosutils.SaxtonBaseNode;
import org.apache.commons.logging.Log;
import org.ros.message.MessageListener;
import org.ros.node.topic.Subscriber;
import org.ros.concurrent.CancellableLoop;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;

/**
 * The Message package is part of the Vehicle Environment package. It processes all V2V and V2I messages coming from Drivers.Comms ROS node.
 * <p>
 *
 *  Command line test: rosrun carma message gov.dot.fhwa.saxton.carma.message.MessageConsumer
 *  rostopic pub /system_alert cav_msgs/SystemAlert type: 5, description: hello}'
 *  rostopic pub /saxton_cav/drivers/arada_application/comms/recv cav_msgs/ByteArray '{messageType: "BSM"}'
 *  rostopic pub /host_bsm cav_msgs/BSM
 */
public class MessageConsumer extends SaxtonBaseNode {

  protected boolean systemReady = false;

  // Publishers
  protected Publisher<SystemAlert> alertPub;
  protected Publisher<cav_msgs.BSM> bsmPub;
  protected Publisher<cav_msgs.MobilityAck> mobilityAckPub;
  protected Publisher<cav_msgs.MobilityGreeting> mobilityGreetingPub;
  protected Publisher<cav_msgs.MobilityIntro> mobilityIntroPub;
  protected Publisher<cav_msgs.MobilityNack> mobilityNAckPub;
//  TODO uncomment when messages are defined
//  protected Publisher<cav_msgs.MobilityPlan> mobilityPlanPub;
//  protected Publisher<cav_msgs.Map> mapPub;
//  protected Publisher<cav_msgs.Spat> spatPub;
//  protected Publisher<cav_msgs.Tim> timPub;

  // Subscribers
  protected Subscriber<cav_msgs.SystemAlert> alertSub;
  protected Subscriber<cav_msgs.BSM> hostBsmSub;
  protected Subscriber<cav_msgs.MobilityAck> mobilityAckOutboundSub;
  protected Subscriber<cav_msgs.MobilityGreeting> mobilityGreetingOutboundSub;
  protected Subscriber<cav_msgs.MobilityIntro> mobilityIntroOutboundSub;
  protected Subscriber<cav_msgs.MobilityNack> mobilityNAckOutboundSub;
//  TODO uncomment when messages are defined
//  protected Subscriber<cav_msgs.MobilityPlan> mobilityPlanOutboundSub;

  @Override
  public GraphName getDefaultNodeName() {
    return GraphName.of("message_consumer");
  }

  @Override
  public void onStart(final ConnectedNode connectedNode) {
    final Log log = connectedNode.getLog();

    // Fake Pubs and Subs TODO: Remove!!!
    // The following are two example pub/subs for connecting to the mock arada driver using the launch file.
    // They should be removed as this process should be handled through the interface manager instead
    final Publisher<cav_msgs.ByteArray> outboundPub = connectedNode.newPublisher("/saxton_cav/drivers/arada_application/comms/outbound", ByteArray._TYPE);
    final Subscriber<cav_msgs.ByteArray> recvSub = connectedNode.newSubscriber("/saxton_cav/drivers/arada_application/comms/recv", ByteArray._TYPE);
    recvSub.addMessageListener(new MessageListener<ByteArray>() {
      @Override public void onNewMessage(ByteArray byteArray) {
        switch (byteArray.getMessageType()) {
          case "BSM":
            log.info("MessageConsumer received ByteArray of type BSM. Publishing BSM message");
            bsmPub.publish(bsmPub.newMessage());
            break;
          case "MobilityAck":
            log.info("MessageConsumer received ByteArray of type MobilityAck. Publishing MobilityAck message");
            mobilityAckPub.publish(mobilityAckPub.newMessage());
            break;
          case "MobilityGreeting":
            log.info("MessageConsumer received ByteArray of type MobilityGreeting. Publishing MobilityGreeting message");
            mobilityGreetingPub.publish(mobilityGreetingPub.newMessage());
            break;
          case "MobilityIntro":
            log.info("MessageConsumer received ByteArray of type MobilityIntro. Publishing MobilityIntro message");
            mobilityIntroPub.publish(mobilityIntroPub.newMessage());
            break;
          case "MobilityNAck":
            log.info("MessageConsumer received ByteArray of type MobilityNack. Publishing MobilityNack message");
            mobilityNAckPub.publish(mobilityNAckPub.newMessage());
            break;
//        TODO uncomment when messages are defined
//        case "MobilityPlan":
//          log.info("MessageConsumer received ByteArray of type MobilityPlan. Publishing MobilityPlan message");
//          mobilityPlanPub.publish(mobilityPlanPub.newMessage());
//          break;
          default:
            log.info("MessageConsumer received ByteArray of type Unknown. Publishing as example BSM message");
            bsmPub.publish(bsmPub.newMessage());
        }
      }
    });



    // Publishers
    alertPub = connectedNode.newPublisher("system_alert", cav_msgs.SystemAlert._TYPE);
    bsmPub = connectedNode.newPublisher("bsm", cav_msgs.BSM._TYPE);
    mobilityAckPub = connectedNode.newPublisher("mobility_ack_recv", cav_msgs.MobilityAck._TYPE);
    mobilityGreetingPub = connectedNode.newPublisher("mobility_greeting_recv", cav_msgs.MobilityGreeting._TYPE);
    mobilityIntroPub = connectedNode.newPublisher("mobility_intro_recv", cav_msgs.MobilityIntro._TYPE);
    mobilityNAckPub = connectedNode.newPublisher("mobility_nack_recv", cav_msgs.MobilityNack._TYPE);
//    TODO uncomment when messages are defined
//    mobilityPlanPub = connectedNode.newPublisher("mobility_plan_recv", cav_msgs.MobilityPlan._TYPE);
//    mapPub = connectedNode.newPublisher("map", cav_msgs.Map._TYPE);
//    spatPub = connectedNode.newPublisher("spat", cav_msgs.Spat._TYPE);
//    timPub = connectedNode.newPublisher("tim", cav_msgs.Tim._TYPE);


    // Subscribers
    alertSub = connectedNode.newSubscriber("system_alert", cav_msgs.SystemAlert._TYPE);
    alertSub.addMessageListener(new MessageListener<cav_msgs.SystemAlert>() {
      @Override
      public void onNewMessage(cav_msgs.SystemAlert message) {
        String messageTypeFullDescription = "NA";

        switch (message.getType()) {
          case cav_msgs.SystemAlert.NOT_READY:
            systemReady = false;
            messageTypeFullDescription = "system not ready alert and will not publish";
            break;
          case cav_msgs.SystemAlert.SYSTEM_READY:
            systemReady = true;
            messageTypeFullDescription = "system ready alert and is beginning to publish";
            break;
          default:
            systemReady = false;
            messageTypeFullDescription = "Unknown system alert type. Assuming system it not ready";
        }
        log.info("message_consumer heard: " + message.getDescription() + "; " + messageTypeFullDescription);
      }
    });//addMessageListener


    hostBsmSub = connectedNode.newSubscriber("host_bsm", cav_msgs.BSM._TYPE);
    hostBsmSub.addMessageListener(new MessageListener<BSM>() {
      @Override public void onNewMessage(BSM bsm) {
        if (systemReady) {
          log.info("MessageConsumer received BSM outbound. Publishing as ByteArray message");
          ByteArray byteArray = outboundPub.newMessage();
          byteArray.setMessageType("BSM"); // Not sure if this is correct type use but will help validate messaging
          outboundPub.publish(byteArray);
        }
      }
    });

    mobilityAckOutboundSub = connectedNode.newSubscriber("mobility_ack_outbound", cav_msgs.MobilityAck._TYPE);
    mobilityAckOutboundSub.addMessageListener(new MessageListener<MobilityAck>() {
      @Override public void onNewMessage(MobilityAck mobilityAck) {
        if (systemReady) {
          log.info("MessageConsumer received BSM outbound. Publishing as ByteArray message");
          ByteArray byteArray = outboundPub.newMessage();
          byteArray.setMessageType("MobilityAck"); // Not sure if this is correct type use but will help validate messaging
          outboundPub.publish(byteArray);
        }
      }
    });

    mobilityGreetingOutboundSub = connectedNode.newSubscriber("mobility_greeting_outbound", cav_msgs.MobilityGreeting._TYPE);
    mobilityGreetingOutboundSub.addMessageListener(new MessageListener<MobilityGreeting>() {
      @Override public void onNewMessage(MobilityGreeting mobilityGreeting) {
        if (systemReady) {
          log.info("MessageConsumer received BSM outbound. Publishing as ByteArray message");
          ByteArray byteArray = outboundPub.newMessage();
          byteArray.setMessageType("MobilityGreeting"); // Not sure if this is correct type use but will help validate messaging
          outboundPub.publish(byteArray);
        }
      }
    });

    mobilityIntroOutboundSub = connectedNode.newSubscriber("mobility_intro_outbound", cav_msgs.MobilityIntro._TYPE);
    mobilityIntroOutboundSub.addMessageListener(new MessageListener<MobilityIntro>() {
      @Override public void onNewMessage(MobilityIntro mobilityIntro) {
        if (systemReady) {
          log.info("MessageConsumer received BSM outbound. Publishing as ByteArray message");
          ByteArray byteArray = outboundPub.newMessage();
          byteArray.setMessageType("MobilityIntro"); // Not sure if this is correct type use but will help validate messaging
          outboundPub.publish(byteArray);
        }
      }
    });

    mobilityNAckOutboundSub = connectedNode.newSubscriber("mobility_nack_outbound", cav_msgs.MobilityNack._TYPE);
    mobilityNAckOutboundSub.addMessageListener(new MessageListener<MobilityNack>() {
      @Override public void onNewMessage(MobilityNack mobilityNack) {
        if (systemReady ) {
          log.info("MessageConsumer received BSM outbound. Publishing as ByteArray message");
          ByteArray byteArray = outboundPub.newMessage();
          byteArray.setMessageType("MobilityNAck"); // Not sure if this is correct type use but will help validate messaging
          outboundPub.publish(byteArray);
        }
      }
    });
//    TODO Uncomment when messages are defined
//    mobilityPlanOutboundSub = connectedNode.newSubscriber("mobility_plan_outbound", cav_msgs.MobilityPlan._TYPE);
//    mobilityPlanOutboundSub.addMessageListener(new MessageListener<MobilityPlan>() {
//      @Override public void onNewMessage(MobilityPlan mobilityPlan) {
//        if (systemReady) {
//          log.info("MessageConsumer received MobilityPlan outbound. Publishing as ByteArray message");
//          ByteArray byteArray = outboundPub.newMessage();
//          byteArray.setMessageType("MobilityPlan"); // Not sure if this is correct type use but will help validate messaging
//          outboundPub.publish(byteArray);
//        }
//      }
//    });

    // This CancellableLoop will be canceled automatically when the node shuts
    // down.
    connectedNode.executeCancellableLoop(new CancellableLoop() {
      private int sequenceNumber;

      @Override
      protected void setup() {
       sequenceNumber = 0;
     }//setup
      @Override
      protected void loop() throws InterruptedException {
        sequenceNumber++;
        Thread.sleep(1000);
      }//loop

   }//CancellableLoop
    );//executeCancellableLoop
  }//onStart
}//AbstractNodeMain

