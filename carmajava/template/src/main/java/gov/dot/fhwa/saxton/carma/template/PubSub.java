/*
 * Copyright (C) 2018-2020 LEIDOS.
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

package gov.dot.fhwa.saxton.carma.template;

import org.ros.message.MessageListener;
import org.ros.node.topic.Subscriber;
import gov.dot.fhwa.saxton.carma.rosutils.SaxtonBaseNode;
import org.ros.concurrent.CancellableLoop;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import gov.dot.fhwa.saxton.carma.rosutils.SaxtonLogger;

/**
 * A simple {@link Publisher} {@link NodeMain}.
 *
 * Replace PubSub with the node name on Column D but using CamelCase.
 *
 * Command line test: rosrun carma template gov.dot.fhwa.saxton.carma.template.PubSub
 */
public class PubSub extends SaxtonBaseNode {

    //TODO: Replace with Column D node name
    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("pub_sub");
    }

  private SaxtonLogger log;

  @Override
    public void onSaxtonStart(final ConnectedNode connectedNode) {

        //final Log log = connectedNode.getLog();
        log = new SaxtonLogger(SaxtonBaseNode.class.getName(), connectedNode.getLog());

        //TODO: Column G topic name
        // Currently setup to listen to it's own message. Change to listen to someone other topic.
        Subscriber<std_msgs.String> subscriber = connectedNode.newSubscriber("template", std_msgs.String._TYPE);

        subscriber.addMessageListener(new MessageListener<std_msgs.String>() {
                                          @Override
                                          public void onNewMessage(std_msgs.String message) {
                                              //TODO: Replace with Column D node name
                                              log.info("TAGNAME", "pub_sub heard: \"" + message.getData() + "\"");
                                          }//onNewMessage
                                      }//MessageListener
        );//addMessageListener

	// Example cav_msgs
	Subscriber<cav_msgs.BSM> bsm_sub = connectedNode.newSubscriber("bsm", cav_msgs.BSM._TYPE);

        //TODO: Column G topic name
        final Publisher<std_msgs.String> publisher =
                connectedNode.newPublisher("template", std_msgs.String._TYPE);


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
                                                     //TODO: Replace with Column D node name
                                                     str.setData("Hello World! " + "I am pub_sub. " + sequenceNumber);
                                                     publisher.publish(str);
                                                     sequenceNumber++;
                                                     Thread.sleep(5000);
                                                 }//loop

                                             }//CancellableLoop
        );//executeCancellableLoop


    }//onStart

  @Override protected void handleException(Throwable e) {

  }
}//SaxtonBaseNode

