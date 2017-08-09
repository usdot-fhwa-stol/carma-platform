/*
 * TODO: Copyright (C) 2017 MaeFromm.
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

package com.github.rosjava.carmajava.message;

import org.apache.commons.logging.Log;
import org.ros.message.MessageListener;
import org.ros.node.topic.Subscriber;

import org.ros.concurrent.CancellableLoop;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;

/**
 * A simple {@link Publisher} {@link NodeMain}.
 *
 * Replace MessageConsumer and message_consumer with another node/topic name.
 */
public class MessageConsumer extends AbstractNodeMain {

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("saxton_cav/vehicle_environment/message");
    }

    @Override
    public void onStart(final ConnectedNode connectedNode) {

        final Log log = connectedNode.getLog();

        //TODO: Currently setup to listen to it's own message. Change to listen to someone other topic.
        Subscriber<std_msgs.String> subscriber = connectedNode.newSubscriber("outbound", std_msgs.String._TYPE);

        subscriber.addMessageListener(new MessageListener<std_msgs.String>() {

                                          @Override
                                          public void onNewMessage(std_msgs.String message) {
                                              log.info("MessageConsumer heard: \"" + message.getData() + "\"");
                                          }//onNewMessage
                                      }//MessageListener
        );//addMessageListener


        final Publisher<std_msgs.String> publisher =
                connectedNode.newPublisher("outbound", std_msgs.String._TYPE);


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
                                                     std_msgs.String str = publisher.newMessage();
                                                     str.setData("Hello World! " + "I am MessageConsumer's Publisher. " + sequenceNumber);
                                                     publisher.publish(str);
                                                     sequenceNumber++;
                                                     Thread.sleep(1000);
                                                 }//loop

                                             }//CancellableLoop
        );//executeCancellableLoop


    }//onStart
}//AbstractNodeMain

