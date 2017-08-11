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

//TODO: Naming convention of "package gov.dot.fhwa.saxton.carmajava.<template>;"
//Originally "com.github.rosjava.carmajava.template;"
package gov.dot.fhwa.saxton.carmajava.template;

import org.apache.commons.logging.Log;
import org.ros.message.MessageListener;
import org.ros.node.topic.Subscriber;

import org.ros.concurrent.CancellableLoop;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;

import org.ros.node.parameter.ParameterTree;
import org.ros.namespace.NameResolver;

import org.ros.message.MessageFactory;

/**
 * A simple {@link Publisher} {@link NodeMain}.
 * <p>
 * Replace PubSub with the node name on Column D but using CamelCase.
 */
public class NodeName extends AbstractNodeMain {

    //TODO: Replace with Column D node name
    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("node_name");
    }

    /*
      private Publisher<tf2_msgs.TFMessage> tfPublisher;
      private Subscriber<tf2_msgs.TFMessage> tfSubscriber;

        tfSubscriber = connectedNode.newSubscriber("tf", tf2_msgs.TFMessage._TYPE);
    tfSubscriber.addMessageListener(new MessageListener<tf2_msgs.TFMessage>() {
      @Override
      public void onNewMessage(tf2_msgs.TFMessage message) {
        counter.incrementAndGet();
      }
    });

    tfPublisher = connectedNode.newPublisher("tf", tf2_msgs.TFMessage._TYPE);

    final tf2_msgs.TFMessage tfMessage = tfPublisher.newMessage();
    geometry_msgs.TransformStamped transformStamped =
        connectedNode.getTopicMessageFactory().newFromType(geometry_msgs.TransformStamped._TYPE);
    tfMessage.getTransforms().add(transformStamped);
    connectedNode.executeCancellableLoop(new CancellableLoop() {
      @Override
      protected void loop() throws InterruptedException {
        tfPublisher.publish(tfMessage);
      }
    });
     */
    @Override
    public void onStart(final ConnectedNode connectedNode) {

        final Log log = connectedNode.getLog();

        //TODO: Column G topic name
        // Currently setup to listen to it's own message. Change to listen to someone other topic.
        Subscriber<cav_msgs.SystemAlert> subscriber = connectedNode.newSubscriber("system_alert", cav_msgs.SystemAlert._TYPE);

        subscriber.addMessageListener(new MessageListener<cav_msgs.SystemAlert>() {
                                          @Override
                                          public void onNewMessage(cav_msgs.SystemAlert message) {

                                              String messageTypeFullDescription = "Not Ready";
                                              /*
                                                uint8   CAUTION = 1
                                                uint8   WARNING = 2
                                                uint8   FATAL = 3
                                                uint8   NOT_READY = 4
                                                uint8   SYSTEM_READY = 5
                                              */

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
                                             // if (message.getType() == cav_msgs.SystemAlert.SYSTEM_READY) {
                                              //    messageTypeFullDescription = "I am Ready! ";
                                              //}

                                              //TODO: Replace with Column D node name
                                              log.info("node_name heard: \"" + message.getDescription() + ";" + messageTypeFullDescription + "\"");
                                              //sensor_msgs.msg.JoyFeedback.TYPE_LED
                                          }//onNewMessage
                                      }//MessageListener
        );//addMessageListener

        //TODO: Column G topic name
        final Publisher<cav_msgs.SystemAlert> systemAlertPublisher =
                connectedNode.newPublisher("system_alert", cav_msgs.SystemAlert._TYPE);


        //Getting the ros param called run_id.
        ParameterTree param = connectedNode.getParameterTree();
        final String rosRunID = param.getString("/run_id");
        //params.setString("~/param_name", test_string);



        /*
        MessageFactory topicMessageFactory = connectedNode.getTopicMessageFactory();


        std_msgs.String systemMessageText = topicMessageFactory.newFromType(std_msgs.String._TYPE);
        systemMessageText.setData("Hello World! " + "I am node_template. " + sequenceNumber + " run_id = " + rosRunID);

        std_msgs.UInt8 systemMessageType =  topicMessageFactory.newFromType(std_msgs.UInt8._TYPE);
        systemMessageType.setUInt8("data", (byte) 5);
        */

        //cav_msgs.SystemAlert systemAlertMsg = topicMessageFactory.newFromType(cav_msgs.SystemAlert._TYPE);
        //systemAlertMsg.setDescription(systemMessageText);
        //systemAlertMsg.setType(systemMessageType);



        /*
        float[] ranges = message.getRanges();
        float northRange = ranges[ranges.length / 2];
        float northEastRange = ranges[ranges.length / 3];
        double linearVelocity = 0.5;
        double angularVelocity = -0.5;
        if (northRange < 1. || northEastRange < 1.) {
            linearVelocity = 0;
            angularVelocity = 0.5;
        }
        twist.getAngular().setZ(angularVelocity);
        twist.getLinear().setX(linearVelocity);
        publisher.publish(twist);
        */

        /*
        //GraphName paramNamespace = GraphName.of(param.getString("parameter_namespace"));
        GraphName myNamespace = GraphName.of(param.getString("system_alert"));

        NameResolver resolver = connectedNode.getResolver().newChild(myNamespace);

        final cav_msgs.SystemAlert system_alert_m =
                topicMessageFactory.newFromType(cav_msgs.SystemAlert._TYPE);
        Map system_alert_map = param.getMap(resolver.resolve("composite"));
        system_alert_m.getType().setW((Double) ((Map) system_alert_map.get("a")).get("w"));
        system_alert_m.getDescription().setX((Double) ((Map) system_alert_map.get("a")).get("x"));
        system_alert_m.getA().setY((Double) ((Map) system_alert_map.get("a")).get("y"));
        system_alert_m.getA().setZ((Double) ((Map) system_alert_map.get("a")).get("z"));
        system_alert_m.getB().setX((Double) ((Map) system_alert_map.get("b")).get("x"));
        system_alert_m.getB().setY((Double) ((Map) system_alert_map.get("b")).get("y"));
        system_alert_m.getB().setZ((Double) ((Map) system_alert_map.get("b")).get("z"));

        param.set(setResolver.resolve("composite"), system_alert_map);
        */

        //RawMessage rawMessage = messageFactory.newFromType("cav_msgs/UInt8");
        //rawMessage.setUInt8("data", (byte) 5);
        //RawMessage rawMessage = messageFactory.newFromType("cav_msgs/UInt8");
        //rawMessage.setUInt8("data", (byte) 5);


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

                                                     //cav_msgs.SystemAlert str = publisher.newMessage();
                                                     //TODO: Replace with Column D node name
                                                     //str.setData("Hello World! " + "I am node_name. " + sequenceNumber);
                                                     //str.setData("Hello World! " + "I am node_template. " + sequenceNumber + " run_id = " + rosRunID);
                                                     //publisher.publish(str);

                                                     cav_msgs.SystemAlert systemAlertMsg = systemAlertPublisher.newMessage();
                                                     systemAlertMsg.setDescription("Hello World! " + "I am node_name. " + sequenceNumber + " run_id = " + rosRunID + ".");
                                                     systemAlertMsg.setType((byte) 5);

                                                     systemAlertPublisher.publish(systemAlertMsg);

                                                     sequenceNumber++;
                                                     Thread.sleep(1000);
                                                 }//loop

                                             }//CancellableLoop
        );//executeCancellableLoop


    }//onStart
}//AbstractNodeMain

