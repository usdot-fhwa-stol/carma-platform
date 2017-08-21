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

import main.java.gov.dot.fhwa.saxton.carma.interfacemgr.InterfaceWorker;
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

import java.util.List;

/**
 * The Interface Manager provides a hardware-agnostic interface to all of the available vehicle
 * hardware devices by searching for all such device drivers and identifying which are successfully
 * connected to their hardware, then facilitating a connection with the appropriate one from the upper
 * level software components based on the desired capabilities.  This main class defines the ROS node
 * and provides all of the ROS communications interfaces.
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
        InterfaceWorker worker = new InterfaceWorker(); //must exist before first message listener







        //jas - add new code here as described below

        ////// topic subscriptions /////

        //create a message listener for /driver_discovery (handleNewDriverStatus)

        //create a message listener for the bond messages of each of the 5 driver types (handleBrokenBond)







        //jas - remove below!
        // Currently setup to listen to it's own message. Change to listen to someone other topic.
        Subscriber<cav_msgs.SystemAlert> subscriber = connectedNode.newSubscriber("system_alert", cav_msgs.SystemAlert._TYPE);

        subscriber.addMessageListener(new MessageListener<cav_msgs.SystemAlert>() {

            @Override
            public void onNewMessage(cav_msgs.SystemAlert message) {

                String messageTypeFullDescription = "NA";

                switch (message.getType()) {
                    case cav_msgs.SystemAlert.CAUTION:
                        messageTypeFullDescription = "Received CAUTION: " + message.getDescription();
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
        });//addMessageListener









        ///// topic publisher /////


        //define our alert publisher as latching so that recipients are guaranteed to see a message even if it is
        // published before the recipient starts up
        final Publisher<cav_msgs.SystemAlert> systemAlertPublisher =
                connectedNode.newPublisher("system_alert", cav_msgs.SystemAlert._TYPE);
        systemAlertPublisher.setLatchMode(true);

        //publish the system not ready message






        //TODO - remove this section
        //Getting the ros param called run_id.
        ParameterTree param = connectedNode.getParameterTree();
        final String rosRunID = param.getString("/run_id");
        //params.setString("~/param_name", param_value);




        // This CancellableLoop will be canceled automatically when the node shuts down
        connectedNode.executeCancellableLoop(new CancellableLoop() {

            @Override
            protected void setup() {
            }//setup

            //Once the wait time expires we declare the system ready for operations, and let
            // all other nodes know.
            @Override
            protected void loop() throws InterruptedException {

                //if system has just achieved operational status then
                    //log it and publish the notification
                    //throw an exception to stop the loop





                //TODO - remove this section

                cav_msgs.SystemAlert message = systemAlertPublisher.newMessage();

                //systemAlertMsg.setDescription("Hello World! " + "I am interface_mgr. " + sequenceNumber + " run_id = " + rosRunID + ".");
                //systemAlertMsg.setType(cav_msgs.SystemAlert.CAUTION);

                systemAlertPublisher.publish(message);






                Thread.sleep(1000);
            }//loop

        });//executeCancellableLoop




        ///// service publisher /////

        //handler for the get_drivers_with_capabilities service (getDrivers)


    }//onStart


    ///// service requestors /////

    /**
     * Binds with the specified driver node.
     *
     * @param driverName - name of the driver's bind topic
     */
    public void bindWithDriver(String driverName) {

        //call the bind service, providing callbacks for both formed bond and broken bond
        //wait for the bond to be formed, and log an error if it is not
    }


    /**
     * Requests the given driver's specific list of data capabilities.
     *
     * @param driverName - name of the driver's api topic
     * @return - a list of data elements available from the driver
     */
    public List<String> getDriverApi(String driverName) {

        //call the api service for the given driver
    }


    /**
     * Handler for a detected broken driver bond - sends an appropriate system alert message.
     * Note that this is not the callback to be provided to the driver's bind service.
     *
     * @param sev - severity of the problem
     * @param message - description of the problem
     */
    public void notifyBrokenBond(AlertSeverity sev, String message) {

        //convert the severity to the appropriate message type
        //set the alert content and send the message
    }
}

