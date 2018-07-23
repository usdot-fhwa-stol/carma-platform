/*
 * Copyright (C) 2018 LEIDOS.
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
import cav_msgs.RobotEnabled;
import cav_msgs.SystemAlert;
import cav_srvs.GetDriverApiResponse;
import cav_srvs.GetDriversWithCapabilitiesResponse;
import gov.dot.fhwa.saxton.carma.rosutils.AlertSeverity;
import gov.dot.fhwa.saxton.carma.rosutils.RosServiceSynchronizer;
import gov.dot.fhwa.saxton.carma.rosutils.SaxtonBaseNode;
import gov.dot.fhwa.saxton.carma.rosutils.SaxtonLogger;
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
import org.ros.node.topic.Subscriber;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;

/**
 * The Interface Manager provides a hardware-agnostic interface to all of the available vehicle
 * hardware devices by searching for all such device drivers and identifying which are successfully
 * connected to their hardware, then facilitating a connection with the appropriate one from the upper
 * level software components based on the desired capabilities.  This main class defines the ROS node
 * and provides all of the ROS communications interfaces.
 *
 * Command line test:
 * rosparam set /interface_mgr/driver_wait_time 20
 * rosrun carma interfacemgr gov.dot.fhwa.saxton.carma.interfacemgr.InterfaceMgr
 */
public class  InterfaceMgr extends SaxtonBaseNode implements IInterfaceMgr {

    protected InterfaceWorker worker_;
    protected SaxtonLogger log_;
    protected ConnectedNode connectedNode_;
    protected CancellableLoop mainLoop_;
    protected boolean robotListenerCreated_ = false;
    protected boolean robotEnabled_ = false; //latch - has robotic control been enabled ever?
    protected boolean shutdownInitiated_ = false;
    protected AtomicBoolean firstRequest = new AtomicBoolean(true);

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("interface_mgr");
    }

    @Override
    public void onSaxtonStart(final ConnectedNode connectedNode) {
        connectedNode_ = connectedNode;
        log_ = new SaxtonLogger(InterfaceMgr.class.getSimpleName(), connectedNode.getLog());
        log_.info("STARTUP", "InterfaceMgr starting up.");

        worker_ = new InterfaceWorker(this, log_); //must exist before first message listener

        //get wait time parameter
        try {
            ParameterTree param = connectedNode.getParameterTree();
            int waitTime = param.getInteger("~/driver_wait_time"); //seconds
            worker_.setWaitTime(waitTime);
            log_.debug("STARTUP", "InterfaceMgr.onStart read waitTime = " + waitTime);
        } catch (Exception e) {
            //do nothing - the worker will use a default value
        }

        ////// topic subscriptions /////

        //create a message listener for /driver_discovery
        Subscriber<cav_msgs.DriverStatus> driverDiscoveryListener = connectedNode.newSubscriber("driver_discovery",
                cav_msgs.DriverStatus._TYPE);
        driverDiscoveryListener.addMessageListener(new MessageListener<DriverStatus>() {

            @Override
            public void onNewMessage(cav_msgs.DriverStatus msg) {
            	try {
	                DriverInfo info = new DriverInfo();
	                info.setName(msg.getName());
	                switch (msg.getStatus()) {
	                case DriverStatus.OFF:
	                    info.setState(DriverState.OFF);
	                    break;
	                case DriverStatus.DEGRADED:
	                    info.setState(DriverState.DEGRADED);
	                    break;
	                case DriverStatus.FAULT:
	                    info.setState(DriverState.FAULT);
	                    break;
	                case DriverStatus.OPERATIONAL:
	                    info.setState(DriverState.OPERATIONAL);
	                    break;
	                default:
	                    info.setState(DriverState.FAULT);
	                }
	                info.setCan(msg.getCanBus());
	                info.setSensor(msg.getSensor());
	                info.setPosition(msg.getPosition());
	                info.setComms(msg.getComms());
	                info.setLonController(msg.getLonController());
	                info.setLatController(msg.getLatController());

	                //add the new driver info to our database
	                worker_.handleNewDriverStatus(info);
            	}catch (Exception e) {
            		handleException(e);
            	}
            }
        });
        
        //message listener for system alerts coming from other nodes
        Subscriber<cav_msgs.SystemAlert> alertListener = connectedNode.newSubscriber("system_alert", cav_msgs.SystemAlert._TYPE);
        alertListener.addMessageListener(new MessageListener<SystemAlert>() {
        	
        	@Override
        	public void onNewMessage(cav_msgs.SystemAlert msg) {
        	    if (msg == null) {
        	        log_.warn("SHUTDOWN", "InterfaceMgr received a NULL system alert. Ignoring.");
        	        return;
                }
        		try {
        		
	        		//if the alert is a FATAL or SHUTDOWN, then proceed to shut down this node
	        		if (msg.getType() == AlertSeverity.FATAL.getVal()  ||
	        			msg.getType() == AlertSeverity.SHUTDOWN.getVal()) {
	        			
	        			String alertType;
	        			if (msg.getType() == AlertSeverity.FATAL.getVal()) {
	        				alertType = "FATAL - ";
	        			}else {
	        				alertType = "SHUTDOWN - ";
	        			}
	        			log_.warn("SHUTDOWN", "InterfaceMgr SHUTTING DOWN after receipt of alert: " + alertType + msg.getDescription());
	        			if(!isShutdownUnderway()) {
	        			    shutdownInitiated_ = true;
	                        connectedNode.shutdown();
	        			}
	        		}
            	}catch (Exception e) {
            		handleException(e);
            	}
        	}
        });
        
        //create a message listener for the bond messages coming from drivers (sendSystemAlert)
        //TODO: this will be implemented in a future iteration due to dependence on
        //      an as-yet non-existent JNI wrapper for the ros bindcpp library.
        //Subscriber<std_msgs.Bond> bondListener =

        //publish the system not ready message
        publishSystemAlert(AlertSeverity.NOT_READY, "System is starting up...", null);

        // This CancellableLoop will be canceled automatically when the node shuts down
        mainLoop_ = new CancellableLoop() {

            //Once the wait time expires we declare the system ready for operations, and let
            // all other nodes know.
            @Override
            protected void loop() throws InterruptedException {

                //attempt to check that we can listen to the controller driver to allow warning others if it dies
                if (!robotListenerCreated_) {
                    checkRobotTopic(connectedNode);
                }

                //if system has just achieved OPERATIONAL status then
                if (worker_.isSystemReady()) {

                    //log it and publish the notification
                    publishSystemAlert(AlertSeverity.DRIVERS_READY, "SYSTEM IS NOW OPERATIONAL", null);
                    log_.info("STARTUP", "///// InterfaceMgr.onStart: all drivers in place -- SYSTEM IS NOW OPERATIONAL");

                    //stop the loop
                    mainLoop_.cancel();
                }

                Thread.sleep(500);
            }//loop

        };
        connectedNode.executeCancellableLoop(mainLoop_);

        ///// service publisher /////

        //handler for the get_drivers_with_capabilities service
        ServiceServer<cav_srvs.GetDriversWithCapabilitiesRequest, cav_srvs.GetDriversWithCapabilitiesResponse> driverCapSvr = connectedNode
                .newServiceServer("get_drivers_with_capabilities", cav_srvs.GetDriversWithCapabilities._TYPE,
                        new ServiceResponseBuilder<cav_srvs.GetDriversWithCapabilitiesRequest, GetDriversWithCapabilitiesResponse>() {

                            @Override
                            public void build(cav_srvs.GetDriversWithCapabilitiesRequest request,
                                    cav_srvs.GetDriversWithCapabilitiesResponse response) {
                            	try {
	                                log_.debug("DRIVER", "InterfaceMgr.driverCapSvr: received request with "
	                                        + request.getCapabilities().size() + " capabilities listed.");
	
	                                //figure out which drivers match the request
	                                List<String> res = worker_.getDrivers(request.getCapabilities());
	                                log_.debug("DRIVER", "InterfaceMgr.driverCapSvr: returning a list of " + res.size()
	                                        + " matching drivers.");
	
	                                //formulate the service response
	                                response.setDriverData(res);
                            	}catch (Exception e) {
                            		handleException(e);
                            	}
                            }
                        });

    }//onSaxtonStart

    ///// service requestors /////

    @Override
    public void bindWithDriver(String driverName) {

        //call the bind service, providing callbacks for both formed bond and broken bond
        String serviceName = driverName + "/bind";
        ServiceClient<cav_srvs.BindRequest, cav_srvs.BindResponse> serviceClient = waitForService(serviceName,
                cav_srvs.Bind._TYPE, connectedNode_, 5000);

        if (serviceClient == null) {
            log_.warn("DRIVER", "InterfaceMgr could not find service \"" + serviceName + "\"");
        }

        //wait for the bond to be formed, and log an error if it is not
        //TODO: wait for the bindcpp JNI wrapper in order to implement this
        log_.info("DRIVER", "InterfaceMgr would now be bound to " + driverName + " but this capability is not yet implemented");
    }

    /**
     * Helper class to allow communication of non-constant data out of the anonymous inner class
     * defined for the getDriverWithApi() method
     */
    protected class ResultHolder {
        private List<String> result;

        void setResult(List<String> res) {
            result = res;
        }

        List<String> getResult() {
            return result;
        }
    }

    @Override
    public List<String> getDriverApi(String driverName) {
        final ResultHolder rh = new ResultHolder();

        //call the api service for the given driver
        final String serviceName = driverName + "/get_driver_api";
        ServiceClient<cav_srvs.GetDriverApiRequest, cav_srvs.GetDriverApiResponse> serviceClient = waitForService(
                serviceName, cav_srvs.GetDriverApi._TYPE, connectedNode_, 5000);

        if (serviceClient == null) {
            log_.warn("DRIVER", "InterfaceMgr could not find service \"" + serviceName + "\"");
        } else {

            cav_srvs.GetDriverApiRequest req = serviceClient.newMessage();

            try {
                RosServiceSynchronizer.callSync(serviceClient, req,
                        new ServiceResponseListener<GetDriverApiResponse>() {
                            @Override
                            public void onSuccess(GetDriverApiResponse response) {
                                rh.setResult(response.getApiList());
                            }

                            @Override
                            public void onFailure(RemoteException e) {
                                log_.warn("DRIVER", "InterfaceMgr.getDriverApi call failed for " + serviceName);
                            }
                        });
            } catch (InterruptedException e) {
                log_.warn("DRIVER", "InterfaceMgr.getDriverApi call failed for " + serviceName);
            }
        }

        return rh.getResult();
    }


    @Override
    public boolean isShutdownUnderway() {
        return shutdownInitiated_;
    }


    /**
     *
     * @param connectedNode
     */
    protected void checkRobotTopic(ConnectedNode connectedNode) {

        //look for the controller driver topic that publishes robotic control status (no point in calling our own ROS service,
        // just make the direct call to the worker class since we have access to it)
        String robotTopic = null;
        List<String> capabilities = new ArrayList<>();
        capabilities.add("robot_status");
        List<String> topics = worker_.getDrivers(capabilities);
        if (topics.size() == 1) {
            robotTopic = topics.get(0);
            robotListenerCreated_ = true;
            log_.info("DRIVER", "InterfaceMgr.checkRobotTopic - capability found, listener being created.");

            //set up listener for the robot topic
            Subscriber<cav_msgs.RobotEnabled> robotListener = connectedNode.newSubscriber(robotTopic, cav_msgs.RobotEnabled._TYPE);
            robotListener.addMessageListener(new MessageListener<RobotEnabled>() {

                @Override
                public void onNewMessage(cav_msgs.RobotEnabled msg) {
                    try {

                        //if robotic control has already been enabled then
                        if (robotEnabled_) {

                            //if robotic control is no longer active then warn and alert all nodes to shut down
                            if (!msg.getRobotActive()) {
                                log_.warn("SHUTDOWN", "InterfaceMgr.robotListener senses robot is no longer active at the hardware level.");
                                publishSystemAlert(AlertSeverity.FATAL, "Robotic control has been disengaged.", null);
                                connectedNode.shutdown();
                            }

                            //else if it is now being commanded then
                        }else if (msg.getRobotEnabled()) {
                            //wait 200 ms to be sure the command has been acted upon, then indicate that it has; at this point
                            // getRobotActive() should return true until it is disengaged, but we can't rely on getRobotEnabled()
                            // as much since it is a command, not an actual status of the hardware
                            try {
                                Thread.sleep(200);
                            }catch (Exception e) {
                            }
                            robotEnabled_ = true;
                            log_.info("DRIVER", "InterfaceMgr.robotListener sensed robot enabled command.");
                        }

                        log_.debug("DRIVER", "InterfaceMgr.robotListener received robot_enabled msg: enabled = "
                                + msg.getRobotEnabled() + ", active = " + msg.getRobotActive());


                    }catch (Exception e) {
                        handleException(e);
                    }
                }
            });

        }else if (topics.size() > 1) { //we didn't find just one topic, so don't know what to do
            log_.warn("DRIVER", "InterfaceMgr search for " + capabilities.get(0) + " topic produced " + topics.size() +
                    " results! CANNOT SENSE DISENGAGEMENT.");
            for (String s : topics) {
                log_.info("DRIVER", "    Topic found: " + s);
            }
            publishSystemAlert(AlertSeverity.CAUTION, "InterfaceMgr unable to sense robotic capability shutdown.", null);

        }else {
            log_.debug("DRIVER", "InterfaceMgr.checkRobotTopic has not yet found a publisher for robot status.");
        }
    }


    /***
     * Handles unhandled exceptions and reports to SystemAlert topic, and log the alert.
     * @param e The exception to handle
     */
    @Override
    protected void handleException(Throwable e) {

        //don't need to log anything here because SaxtonBaseNode handler has already done that

        //if it has been less than 2 min since we declared the system to be ready for operation then
        if (worker_.timeSinceSystemReady() < 2*60*1000) {
            publishSystemAlert(AlertSeverity.FATAL, "Unknown exception trapped in InterfaceMgr - COMMANDING SYSTEM SHUT DOWN.", e);
        }else {
            publishSystemAlert(AlertSeverity.WARNING, "Unknown exception trapped in InterfaceMgr - shutting down myself only.", e);
        }

        shutdownInitiated_ = true;
        connectedNode_.shutdown();
    }

    @Override
    public void errorShutdown(String msg) {
        shutdownInitiated_ = true;
        publishSystemAlert(AlertSeverity.FATAL, msg, null);
        connectedNode_.shutdown();
    }

}
