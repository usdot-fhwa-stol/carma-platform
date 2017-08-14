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
package gov.dot.fhwa.saxton.carma.interfacemgr;

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
 * The Interface Manager provides a hardware-agnostic interface to all of the available vehicle
 * hardware devices by searching for all such device drivers and identifying which are successfully
 * connected to their hardware, then facilitating a connection with the appropriate one from the upper
 * level software components based on the desired capabilities.
  * <p>
 *
 * Command line test: rosrun carma interfacemgr gov.dot.fhwa.saxton.carma.interfacemgr.InterfaceMgr
 */
public class InterfaceMgr extends AbstractNodeMain {

  @Override
  public GraphName getDefaultNodeName() {
    return GraphName.of("interface_mgr");
  }

  @Override
  public void onStart(final ConnectedNode connectedNode) {

    final Log log = connectedNode.getLog();

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
                                        case cav_msgs.SystemAlert.SYSTEM_READY:
                                          messageTypeFullDescription = "I am Ready! ";
                                          break;
                                        default:
                                          messageTypeFullDescription = "I am NOT Ready! ";
                                      }

                                      log.info("interface_mgr heard: \"" + message.getDescription() + ";" + messageTypeFullDescription + "\"");

                                    }//onNewMessage
                                  }//MessageListener
    );//addMessageListener

    final Publisher<cav_msgs.SystemAlert> systemAlertPublisher =
      connectedNode.newPublisher("system_alert", cav_msgs.SystemAlert._TYPE);


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

                                             cav_msgs.SystemAlert systemAlertMsg = systemAlertPublisher.newMessage();
                                             systemAlertMsg.setDescription("Hello World! " + "I am interface_mgr. " + sequenceNumber + " run_id = " + rosRunID + ".");
                                             systemAlertMsg.setType(cav_msgs.SystemAlert.SYSTEM_READY);

                                             systemAlertPublisher.publish(systemAlertMsg);

                                             sequenceNumber++;
                                             Thread.sleep(1000);
                                           }//loop

                                         }//CancellableLoop
    );//executeCancellableLoop
  }//onStart
}//AbstractNodeMain

