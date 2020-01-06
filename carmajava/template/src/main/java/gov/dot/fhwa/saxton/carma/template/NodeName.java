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

import gov.dot.fhwa.saxton.carma.rosutils.AlertSeverity;
import org.ros.message.MessageListener;
import org.ros.node.topic.Subscriber;
import gov.dot.fhwa.saxton.carma.rosutils.SaxtonBaseNode;
import org.ros.concurrent.CancellableLoop;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.node.parameter.ParameterTree;
import gov.dot.fhwa.saxton.carma.rosutils.SaxtonLogger;

/**
 * A simple {@link Publisher} {@link NodeMain}.
 * <p>
 * Replace PubSub with the node name on Column D but using CamelCase.
 *
 * Command line test: rosrun carma template gov.dot.fhwa.saxton.carma.template.NodeName
 */
public class NodeName extends SaxtonBaseNode {

  private SaxtonLogger log;

  //TODO: Replace with Column D node name
  @Override
  public GraphName getDefaultNodeName() {
    return GraphName.of("node_name");
  }

  @Override
  public void onSaxtonStart(final ConnectedNode connectedNode) {

    log = new SaxtonLogger(NodeName.class.getSimpleName(), connectedNode.getLog());


    //TODO: Column G topic name
    // Currently setup to listen to it's own message. Change to listen to someone other topic.
    Subscriber<cav_msgs.SystemAlert> subscriber = connectedNode.newSubscriber("system_alert", cav_msgs.SystemAlert._TYPE);

    subscriber.addMessageListener(new MessageListener<cav_msgs.SystemAlert>() {
                                    @Override
                                    public void onNewMessage(cav_msgs.SystemAlert message) {

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
                                        case cav_msgs.SystemAlert.DRIVERS_READY:
                                          messageTypeFullDescription = "I am Ready! ";
                                          break;
                                        default:
                                          messageTypeFullDescription = "I am NOT Ready! ";
                                      }

                                      //TODO: Replace with Column D node name
                                      log.info("TAGNAME", "node_name heard: \"" + message.getDescription() + ";" + messageTypeFullDescription + "\"");

                                    }//onNewMessage
                                  }//MessageListener
    );//addMessageListener

    //Getting the ros param called run_id.
    ParameterTree param = connectedNode.getParameterTree();
    final String rosRunID = param.getString("/run_id");
    //params.setString("~/param_name", param_value);

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

                                             publishSystemAlert(AlertSeverity.DRIVERS_READY, "Hello World! " + "I am node_name. " + sequenceNumber + " run_id = " + rosRunID + ".", null );

                                             sequenceNumber++;
                                             Thread.sleep(5000);
                                           }//loop

                                         }//CancellableLoop
    );//executeCancellableLoop
  }//onStart

  @Override protected void handleException(Throwable e) {

  }
}//SaxtonBaseNode

