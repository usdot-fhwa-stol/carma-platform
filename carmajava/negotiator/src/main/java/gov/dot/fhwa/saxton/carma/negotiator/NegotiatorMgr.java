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

package gov.dot.fhwa.saxton.carma.negotiator;

import gov.dot.fhwa.saxton.carma.rosutils.SaxtonBaseNode;
import org.apache.commons.logging.Log;
import org.ros.message.MessageListener;
import org.ros.message.MessageFactory;
import org.ros.node.topic.Subscriber;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.node.parameter.ParameterTree;
import org.ros.concurrent.CancellableLoop;
import org.ros.namespace.GraphName;
import org.ros.namespace.NameResolver;

/**
 * The Negotiator package responsibility is to manage the details of negotiating tactical and strategic
 * agreements between the host vehicle and any other transportation system entities.
 * <p>
 * Command line test: rosrun carmajava negotiator gov.dot.fhwa.saxton.carma.negotiator.NegotiatorMgr
 */
public class NegotiatorMgr extends SaxtonBaseNode {

  @Override public GraphName getDefaultNodeName() {
    return GraphName.of("negotiator_mgr");
  }

  @Override public void onStart(final ConnectedNode connectedNode) {

    final Log log = connectedNode.getLog();

    // Topics
    // Publishers
    final Publisher<cav_msgs.MobilityAck> mobAckOutPub =
      connectedNode.newPublisher("mobility_ack_outbound", cav_msgs.MobilityAck._TYPE);
    final Publisher<cav_msgs.MobilityGreeting> mobGreetOutPub =
      connectedNode.newPublisher("mobility_greeting_outbound", cav_msgs.MobilityGreeting._TYPE);
    final Publisher<cav_msgs.MobilityIntro> mobIntroOutPub =
      connectedNode.newPublisher("mobility_intro_outbound", cav_msgs.MobilityIntro._TYPE);
    final Publisher<cav_msgs.MobilityNack> mobNackOutPub =
      connectedNode.newPublisher("mobility_nack_outbound", cav_msgs.MobilityNack._TYPE);
//    TODO: Uncomment when the MobilityPlan message is implemented
//    final Publisher<cav_msgs.MobilityPlan> mobPlanOutPub =
//      connectedNode.newPublisher("mobility_plan_outbound", cav_msgs.MobilityPlan._TYPE);
    final Publisher<cav_msgs.NewPlan> newPlanInPub =
      connectedNode.newPublisher("new_plan_inbound", cav_msgs.NewPlan._TYPE);
//    TODO: Uncomment when the PlanStatus message is implemented.
//    final Publisher<cav_msgs.PlanStatus> planStatusPub =
//      connectedNode.newPublisher("plan_status", cav_msgs.PlanStatus._TYPE);
    final Publisher<cav_msgs.SystemAlert> systemAlertPub =
      connectedNode.newPublisher("system_alert", cav_msgs.SystemAlert._TYPE);

    // Subscribers
    Subscriber<cav_msgs.NewPlan> newPlanOutSub =
      connectedNode.newSubscriber("new_plan_outbound", cav_msgs.NewPlan._TYPE);
    Subscriber<cav_msgs.MobilityAck> mobAckInSub =
      connectedNode.newSubscriber("mobility_ack_inbound", cav_msgs.MobilityAck._TYPE);
    Subscriber<cav_msgs.MobilityGreeting> mobGreetInSub =
      connectedNode.newSubscriber("mobility_greeting_inbound", cav_msgs.MobilityGreeting._TYPE);
    Subscriber<cav_msgs.MobilityIntro> mobIntroInSub =
      connectedNode.newSubscriber("mobility_intro_inbound", cav_msgs.MobilityIntro._TYPE);
    Subscriber<cav_msgs.MobilityNack> mobNackInSub =
      connectedNode.newSubscriber("mobility_nack_inbound", cav_msgs.MobilityNack._TYPE);
//    TODO: Uncomment when the MobilityPlan message is implemented
//    Subscriber<cav_msgs.MobilityPlan> mobPlanInSub =
//      connectedNode.newSubscriber("mobility_plan_inbound", cav_msgs.MobilityPlan._TYPE);
    Subscriber<cav_msgs.SystemAlert> alertSub =
      connectedNode.newSubscriber("system_alert", cav_msgs.SystemAlert._TYPE);

    alertSub.addMessageListener(new MessageListener<cav_msgs.SystemAlert>() {
      @Override public void onNewMessage(cav_msgs.SystemAlert message) {

        String messageTypeFullDescription = "NA";

        switch (message.getType()) {
          case cav_msgs.SystemAlert.CAUTION:
            messageTypeFullDescription = "Take caution! ";
            break;
          case cav_msgs.SystemAlert.WARNING:
            messageTypeFullDescription = "I have a warning! ";
            break;
          case cav_msgs.SystemAlert.FATAL:
            messageTypeFullDescription = "I am FATAL! ";
            break;
          case cav_msgs.SystemAlert.NOT_READY:
            messageTypeFullDescription = "I am NOT Ready! ";
            break;
          case cav_msgs.SystemAlert.SYSTEM_READY:
            messageTypeFullDescription = "I am Ready! ";
            break;
          default:
            messageTypeFullDescription = "I am NOT Ready! ";
        }

        log.info(
          "negotiator_mgr heard: \"" + message.getDescription() + ";" + messageTypeFullDescription
            + "\"");

      }//onNewMessage
    }//MessageListener
    );//addMessageListener

    //Getting the ros param called run_id.
    ParameterTree param = connectedNode.getParameterTree();
    final String rosRunID = param.getString("/run_id");

    // This CancellableLoop will be canceled automatically when the node shuts down
    connectedNode.executeCancellableLoop(new CancellableLoop() {
      private int sequenceNumber;

      @Override protected void setup() {
        sequenceNumber = 0;
      }//setup

      @Override protected void loop() throws InterruptedException {

        cav_msgs.SystemAlert systemAlertMsg = systemAlertPub.newMessage();
        systemAlertMsg.setDescription(
          "Hello World! " + "I am negotiator_mgr. " + sequenceNumber + " run_id = " + rosRunID
            + ".");
        systemAlertMsg.setType(cav_msgs.SystemAlert.SYSTEM_READY);

        systemAlertPub.publish(systemAlertMsg);

        sequenceNumber++;
        Thread.sleep(30000);
      }//loop

    });//executeCancellableLoop
  }//onStart
}//AbstractNodeMain

