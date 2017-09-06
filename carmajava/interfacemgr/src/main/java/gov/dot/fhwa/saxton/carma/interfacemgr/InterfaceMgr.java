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
import cav_msgs.SystemAlert;
import cav_srvs.GetDriverApiResponse;
import cav_srvs.GetDriversWithCapabilitiesRequest;
import cav_srvs.GetDriversWithCapabilitiesResponse;
import gov.dot.fhwa.saxton.carma.rosutils.SaxtonBaseNode;
import org.apache.commons.logging.Log;
import org.ros.concurrent.CancellableLoop;
import org.ros.exception.RemoteException;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.parameter.ParameterTree;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseBuilder;
import org.ros.node.service.ServiceResponseListener;
import org.ros.node.service.ServiceServer;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import java.util.List;

/**
 * The Interface Manager provides a hardware-agnostic interface to all of the available vehicle
 * hardware devices by searching for all such device drivers and identifying which are successfully
 * connected to their hardware, then facilitating a connection with the appropriate one from the upper
 * level software components based on the desired capabilities.  This main class defines the ROS node
 * and provides all of the ROS communications interfaces.
 *
 * Command line test:
 * rosparam set /interface_mgr/driver_wait_time 10
 * rosrun carma interfacemgr gov.dot.fhwa.saxton.carma.interfacemgr.InterfaceMgr
 */
public class InterfaceMgr extends SaxtonBaseNode implements IInterfaceMgr {

    protected InterfaceWorker   worker_;
    protected Log               log_;
    protected ConnectedNode     connectedNode_;
    protected Publisher<cav_msgs.SystemAlert> systemAlertPublisher_;

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("interface_mgr");
    }


    @Override
    public void onStart(final ConnectedNode connectedNode) {
        connectedNode_ = connectedNode;
        log_ = connectedNode.getLog();
        log_.info("InterfaceMgr starting up.");

        worker_ = new InterfaceWorker(this, log_); //must exist before first message listener

        //get wait time parameter
        try {
            ParameterTree param = connectedNode.getParameterTree();
            int waitTime = param.getInteger("~/driver_wait_time"); //seconds
            worker_.setWaitTime(waitTime);
            log_.debug("InterfaceMgr.onStart read waitTime = " + waitTime);
        }catch (Exception e){
            //do nothing - the worker will use a default value
        }



        ////// topic subscriptions /////

        //create a message listener for /driver_discovery
        Subscriber<cav_msgs.DriverStatus> driverDiscoveryListener =
                connectedNode.newSubscriber("driver_discovery", cav_msgs.DriverStatus._TYPE);
        driverDiscoveryListener.addMessageListener(new MessageListener<DriverStatus>() {

            @Override
            public void onNewMessage(cav_msgs.DriverStatus msg) {
                DriverInfo info = new DriverInfo();
                info.setName(msg.getName());
                switch (msg.getStatus()) {
                    case DriverStatus.OFF:          info.setState(DriverState.OFF);            break;
                    case DriverStatus.DEGRADED:     info.setState(DriverState.DEGRADED);       break;
                    case DriverStatus.FAULT:        info.setState(DriverState.FAULT);          break;
                    case DriverStatus.OPERATIONAL:  info.setState(DriverState.OPERATIONAL);    break;
                    default:
                        info.setState(DriverState.FAULT);
                }
                info.setCan(msg.getCanBus());
                info.setSensor(msg.getSensor());
                info.setPosition(msg.getPosition());
                info.setComms(msg.getComms());
                info.setController(msg.getController());
                log_.debug("InterfaceMgr.driverDiscoveryListener received new status: " + info.getName()
                            + ", " + info.getState().toString());

                //add the new driver info to our database
                worker_.handleNewDriverStatus(info);
            }
        });

        //create a message listener for the bond messages coming from drivers (sendSystemAlert)
            //TODO: this will be implemented in a future iteration due to dependence on
            //      an as-yet non-existent JNI wrapper for the ros bindcpp library.
        //Subscriber<std_msgs.Bond> bondListener =







        ///// topic publisher /////


        //define our alert publisher as latching so that recipients are guaranteed to see a message even if it is
        // published before the recipient starts up
        systemAlertPublisher_ = connectedNode.newPublisher("system_alert", cav_msgs.SystemAlert._TYPE);
        systemAlertPublisher_.setLatchMode(true);

        //publish the system not ready message
        sendSystemAlert(AlertSeverity.NOT_READY, "System is starting up...");

        // This CancellableLoop will be canceled automatically when the node shuts down
        connectedNode.executeCancellableLoop(new CancellableLoop() {

            //Once the wait time expires we declare the system ready for operations, and let
            // all other nodes know.
            @Override
            protected void loop() throws InterruptedException {

                //if system has just achieved OPERATIONAL status then
                if (worker_.isSystemReady()) {

                    //log it and publish the notification
                    sendSystemAlert(AlertSeverity.SYSTEM_READY, "SYSTEM IS NOW OPERATIONAL");
                    log_.info("///// InterfaceMgr.onStart: all drivers in place -- SYSTEM IS NOW OPERATIONAL");

                    //stop the loop
                    throw new InterruptedException("System is OPERATIONAL. Stopping loop.");
                }

                Thread.sleep(1000);
            }//loop

        });//executeCancellableLoop


        ///// service publisher /////

        //handler for the get_drivers_with_capabilities service
        ServiceServer<cav_srvs.GetDriversWithCapabilitiesRequest, cav_srvs.GetDriversWithCapabilitiesResponse> driverCapSvr =
                connectedNode.newServiceServer("get_drivers_with_capabilities", cav_srvs.GetDriversWithCapabilities._TYPE,
                new ServiceResponseBuilder<cav_srvs.GetDriversWithCapabilitiesRequest, GetDriversWithCapabilitiesResponse>() {

            @Override
            public void build(cav_srvs.GetDriversWithCapabilitiesRequest request,
                              cav_srvs.GetDriversWithCapabilitiesResponse response) {

                log_.debug("InterfaceMgr.driverCapSvr: received request with " + request.getCapabilities().size() +
                        " capabilities listed.");

                //figure out which drivers match the request
                List<String> res = worker_.getDrivers(request.getCapabilities());
                log_.debug("InterfaceMgr.driverCapSvr: returning a list of " + res.size() + " matching drivers.");

                //formulate the service response
                response.setDriverData(res);
            }
        });

    }//onStart


    ///// service requestors /////

    @Override
    public void bindWithDriver(String driverName) {

        //call the bind service, providing callbacks for both formed bond and broken bond
        String serviceName = driverName + "/bind";
        ServiceClient<cav_srvs.BindRequest, cav_srvs.BindResponse> serviceClient =
                waitForService(serviceName, cav_srvs.Bind._TYPE, connectedNode_, 5000);

        if (serviceClient == null) {
            log_.warn("InterfaceMgr could not find service \"" + serviceName + "\"");
        }

        //wait for the bond to be formed, and log an error if it is not
        //TODO: wait for the bindcpp JNI wrapper in order to implement this
        log_.info("InterfaceMgr would now be bound to " + driverName + " but this capability is not yet implemented");
    }


    /**
     * Helper class to allow communication of non-constant data out of the anonymous inner class
     * defined for the getDriverWithApi() method
     */
    protected class ResultHolder {
        private List<String> result;

        void setResult(List<String> res) { result = res; }
        List<String> getResult() {return result; }
    }


    @Override
    public List<String> getDriverApi(String driverName) {
        final ResultHolder rh = new ResultHolder();

        //call the api service for the given driver
        final String serviceName = driverName + "/get_driver_api";
        ServiceClient<cav_srvs.GetDriverApiRequest, cav_srvs.GetDriverApiResponse> serviceClient =
                waitForService(serviceName, cav_srvs.GetDriverApi._TYPE, connectedNode_, 5000);

        if (serviceClient == null) {
            log_.warn("InterfaceMgr could not find service \"" + serviceName + "\"");
        }else {

            cav_srvs.GetDriverApiRequest req = serviceClient.newMessage();

            serviceClient.call(req, new ServiceResponseListener<GetDriverApiResponse>() {
                @Override
                public void onSuccess(GetDriverApiResponse response) {
                    rh.setResult(response.getApiList());
                }

                @Override
                public void onFailure(RemoteException e) {
                    log_.warn("InterfaceMgr.getDriverApi call failed for " + serviceName);
                }
            });
        }

        return rh.getResult();
    }

    @Override
    public void sendSystemAlert(AlertSeverity sev, String message) {

        //in case this method gets called before our onStart is complete, need to check for valid publisher
        if (systemAlertPublisher_ != null) {

            //convert the severity to the appropriate message type
            SystemAlert alert = systemAlertPublisher_.newMessage();
            alert.setType((byte)sev.getVal());

            //set the alert content and send the message
            alert.setDescription(message);

            systemAlertPublisher_.publish(alert);
            log_.info("InterfaceMgr.sendSystemAlert: " + alert.toString() + ", " + message);
        }
    }
}

