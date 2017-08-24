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

package gov.dot.fhwa.saxton.carma.interfacemgr;

import cav_msgs.DriverStatus;
import gov.dot.fhwa.saxton.carma.rosutils.SaxtonBaseNode;
import org.apache.commons.logging.Log;
import org.ros.message.MessageListener;
import org.ros.node.topic.Subscriber;
import org.ros.concurrent.CancellableLoop;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;

import org.ros.node.parameter.ParameterTree;

import java.util.ArrayList;
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
public class InterfaceMgr extends SaxtonBaseNode implements IInterfaceMgr {

    private InterfaceWorker     worker_ = new InterfaceWorker(this);
    private Log                 log_;


    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("interface_mgr");
    }

    @Override
    public void onStart(final ConnectedNode connectedNode) {
        InterfaceWorker worker = new InterfaceWorker(this); //must exist before first message listener
        log_ = connectedNode.getLog();






        //todo - add new code here as described below

        ////// topic subscriptions /////

        //create a message listener for /driver_discovery (handleNewDriverStatus)
        Subscriber<cav_msgs.DriverStatus> driverDiscoveryListener =
                connectedNode.newSubscriber("driver_discovery", cav_msgs.DriverStatus._TYPE);
        driverDiscoveryListener.addMessageListener(new MessageListener<DriverStatus>() {

            @Override
            public void onNewMessage(cav_msgs.DriverStatus msg) {
                DriverInfo info = new DriverInfo();
                info.setName(msg.getName());
                switch (msg.getStatus()) {
                    case DriverStatus.OFF:          info.setStatus(DriverState.off);            break;
                    case DriverStatus.DEGRADED:     info.setStatus(DriverState.degraded);       break;
                    case DriverStatus.FAULT:        info.setStatus(DriverState.fault);          break;
                    case DriverStatus.OPERATIONAL:  info.setStatus(DriverState.operational);    break;
                    default:
                        info.setStatus(DriverState.fault);
                }
                info.setCan(msg.getCanBus());
                info.setSensor(msg.getSensor());
                info.setPosition(msg.getPosition());
                info.setComms(msg.getComms());
                info.setController(msg.getController());
                log_.debug("InterfaceMgr.driverDiscoveryListener received new status: " + info.getName()
                            + ", " + info.getStatus().toString());

                worker_.handleNewDriverStatus(info);
            }
        });

        //create a message listener for the bond messages coming from drivers (handleBrokenBond)
            //NOTE: this will be implemented in a future iteration due to dependence on
            //      an as-yet non-existent JNI wrapper for the ros bindcpp library.







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

    @Override
    public void bindWithDriver(String driverName) {

        //call the bind service, providing callbacks for both formed bond and broken bond
        //wait for the bond to be formed, and log an error if it is not
    }


    @Override
    public List<String> getDriverApi(String driverName) {

        //call the api service for the given driver

        return new ArrayList<String>(); //TODO - bogus
    }


    @Override
    public void notifyBrokenBond(AlertSeverity sev, String message) {

        //convert the severity to the appropriate message type
        //set the alert content and send the message
    }
}

