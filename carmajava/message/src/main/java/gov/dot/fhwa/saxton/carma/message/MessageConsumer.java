/*
 * Copyright (C) 2017 LEIDOS.
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

package gov.dot.fhwa.saxton.carma.message;

import cav_msgs.*;
import cav_srvs.*;
import gov.dot.fhwa.saxton.carma.factory.FactoryManager;
import gov.dot.fhwa.saxton.carma.factory.IMessageFactory;
import gov.dot.fhwa.saxton.carma.factory.MessageContainer;
import gov.dot.fhwa.saxton.carma.rosutils.AlertSeverity;
import gov.dot.fhwa.saxton.carma.rosutils.SaxtonBaseNode;
import gov.dot.fhwa.saxton.carma.rosutils.SaxtonLogger;
import gov.dot.fhwa.saxton.carma.rosutils.RosServiceSynchronizer;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;

import org.ros.message.MessageListener;
import org.ros.node.topic.Subscriber;
import org.ros.concurrent.CancellableLoop;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;

//Services
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;
import org.ros.exception.RemoteException;
import org.ros.exception.RosRuntimeException;
import org.ros.exception.ServiceNotFoundException;

/*
 * The Message package is part of the Vehicle Environment package.
 * It processes all V2V and V2I messages coming from Drivers.Comms ROS node.
 * Command line test: rosrun carma message gov.dot.fhwa.saxton.carma.message.MessageConsumer
 * rostopic pub /system_alert cav_msgs/SystemAlert '{type: 5, description: hello}'
 * rosparam set /interface_mgr/driver_wait_time 10
 * rosrun carma interfacemgr gov.dot.fhwa.saxton.carma.interfacemgr.InterfaceMgr
 * rostopic pub /saxton_cav/drivers/arada_application/comms/recv cav_msgs/ByteArray '{messageType: "BSM"}'
 * rostopic pub /host_bsm cav_msgs/BSM '{}'
 */

public class MessageConsumer extends SaxtonBaseNode {

	protected boolean driversReady = false;

	// Publishers
	protected Publisher<ByteArray> outboundPub_; //outgoing byte array, after encode
	protected Publisher<BSM> bsmPub_; //incoming BSM, after decoded
	// protected Publisher<cav_msgs.MobilityAck> mobilityAckPub;
	// protected Publisher<cav_msgs.MobilityGreeting> mobilityGreetingPub;
	// protected Publisher<cav_msgs.MobilityIntro> mobilityIntroPub;
	// protected Publisher<cav_msgs.MobilityNack> mobilityNAckPub;
	// protected Publisher<cav_msgs.MobilityPlan> mobilityPlanPub;
	// protected Publisher<cav_msgs.Map> mapPub;
	// protected Publisher<cav_msgs.Spat> spatPub;
	// protected Publisher<cav_msgs.Tim> timPub;

	// Subscribers
	protected Subscriber<SystemAlert> alertSub_;
	protected Subscriber<ByteArray> inboundSub_; //incoming byte array, need to decode
	protected Subscriber<BSM> bsmSub_; //outgoing BSM, need to encode
	// protected Subscriber<cav_msgs.MobilityAck> mobilityAckOutboundSub;
	// protected Subscriber<cav_msgs.MobilityGreeting> mobilityGreetingOutboundSub;
	// protected Subscriber<cav_msgs.MobilityIntro> mobilityIntroOutboundSub;
	// protected Subscriber<cav_msgs.MobilityNack> mobilityNAckOutboundSub;
	// protected Subscriber<cav_msgs.MobilityPlan> mobilityPlanOutboundSub;

	// Used Services
	protected ServiceClient<GetDriversWithCapabilitiesRequest, GetDriversWithCapabilitiesResponse> getDriversWithCapabilitiesClient_;
	
	//Log for this node
    protected SaxtonLogger log_ = null;
	
	//Connected Node
	protected ConnectedNode connectedNode_ = null;
	
	//For recoding message frequency
	protected final int sample_window = 5;
	protected double last_outgoing_sample_time = 0;
	protected int outgoing_bsm_counter = 0;
	protected double last_incoming_sample_time = 0;
	protected int incoming_bsm_counter = 0;
	
	protected BlockingQueue<MessageContainer<?>> dsrcMessageQueue = new LinkedBlockingQueue<>();
	
	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("message_consumer");
	}

	@Override
	public void onSaxtonStart(final ConnectedNode connectedNode) {
		
		//initialize connectedNode and log
		this.connectedNode_ = connectedNode;
		this.log_ = new SaxtonLogger(MessageConsumer.class.getSimpleName(), this.connectedNode_.getLog());
		
		//initialize alert sub, pub
		alertSub_ = this.connectedNode_.newSubscriber("system_alert", SystemAlert._TYPE);
		if(alertSub_ == null) {
		    log_.warn("Cannot initialize system_alert subscriber!");
		    throw new RosRuntimeException("system_alert subscriber not found!");
		}
		alertSub_.addMessageListener(new MessageListener<SystemAlert>() {
			@Override
			public void onNewMessage(SystemAlert message) {
				if(message.getType() == SystemAlert.FATAL || message.getType() == SystemAlert.SHUTDOWN) {
					connectedNode_.shutdown();
				} else if(message.getType() == SystemAlert.DRIVERS_READY) {
					driversReady = true;
				}
			}
		});
		
		//Use cav_srvs.GetDriversWithCapabilities and wait for driversReady signal
		getDriversWithCapabilitiesClient_ = this.waitForService("get_drivers_with_capabilities", GetDriversWithCapabilities._TYPE, connectedNode, 10000);
		if(getDriversWithCapabilitiesClient_ == null) {
			log_.warn("Cannot find service get_drivers_with_capabilities");
			try {
                throw new ServiceNotFoundException("get_drivers_with_capabilities is not found!");
            } catch (ServiceNotFoundException e) {
                handleException(e);
            }
		}
		
		//Wait for driversReady
		while(!driversReady) {
		    try {
		        Thread.sleep(1000);
		    } catch(InterruptedException e) {
		        Thread.interrupted();
		    }
		}
		
		//Make service calls and validate drivers information
		GetDriversWithCapabilitiesRequest request = getDriversWithCapabilitiesClient_.newMessage();
		request.setCapabilities(Arrays.asList("inbound_binary_msg", "outbound_binary_msg"));
		final List<GetDriversWithCapabilitiesResponse> driver_response = new ArrayList<>(1);
		try {
            RosServiceSynchronizer.callSync(getDriversWithCapabilitiesClient_, request, new ServiceResponseListener<GetDriversWithCapabilitiesResponse>() {
            	
            	@Override
            	public void onSuccess(GetDriversWithCapabilitiesResponse response) {
            	    driver_response.set(0, response);
            		log_.info("GetDriversWithCapabilitiesResponse: " + driver_response.get(0).getDriverData());
            	}
            	
            	@Override
            	public void onFailure(RemoteException e) {
            		throw new RosRuntimeException(e);
            	}
            	
            });
        } catch (InterruptedException e1) {
            handleException(e1);
        }
		
		if(driver_response.get(0).getDriverData().size() == 0) {
		    log_.warn("Cannot find drivers.");
		    throw new RosRuntimeException("Cannot find drivers.");
		}
		
		String J2735_inbound_binary_msg = null, J2735_outbound_binary_msg = null;
		for(String s : driver_response.get(0).getDriverData()) {
			if(s.endsWith("/dsrc/comms/inbound_binary_msg")) {
				J2735_inbound_binary_msg = s;
			} else if(s.endsWith("/dsrc/comms/outbound_binary_msg")) {
				J2735_outbound_binary_msg = s;
			}
		}
		
		if(J2735_inbound_binary_msg == null || J2735_outbound_binary_msg == null) {
			log_.error("Unable to find suitable dsrc driver!");
			throw new RosRuntimeException("Unable to find suitable dsrc driver!");
		}
		
		//initialize Pubs
		bsmPub_ = connectedNode_.newPublisher("incoming_bsm", BSM._TYPE);
		outboundPub_ = connectedNode_.newPublisher(J2735_outbound_binary_msg, ByteArray._TYPE);
		if(bsmPub_ == null || outboundPub_ == null) {
		    log_.warn("Cannot initialize subPub or outboundPub.");
            throw new RosRuntimeException("Cannot initialize publishers.");
		}
		
		//initialize Subs
		bsmSub_ = connectedNode_.newSubscriber("outgoing_bsm", BSM._TYPE);
		if(bsmSub_ == null) {
		    log_.warn("Cannot initialize bsmSub");
		    throw new RosRuntimeException("Cannot initialize subscribers.");
		}
		bsmSub_.addMessageListener((bsm)-> {
		    dsrcMessageQueue.add(new MessageContainer<BSM>("BSM", bsm));
		    sampleEncodedBSM();
		});

		inboundSub_ = connectedNode_.newSubscriber(J2735_inbound_binary_msg, ByteArray._TYPE);
		inboundSub_.addMessageListener((msg) -> {
	        sendMessage(FactoryManager.getMessageFactory(msg.getMessageType(), connectedNode_, log_, connectedNode_.getTopicMessageFactory()).decode(msg));
		});
		
		// This CancellableLoop will be canceled automatically when the node shuts down.
		connectedNode_.executeCancellableLoop(new CancellableLoop() {
			@Override
			protected void loop() throws InterruptedException {
			    MessageContainer<?> outgoingMessage = dsrcMessageQueue.take();
				IMessageFactory<?> factory = FactoryManager.getMessageFactory(outgoingMessage.getType(), connectedNode_, log_, connectedNode_.getTopicMessageFactory());
				sendMessage(factory.encode(outgoingMessage));
			}
		});
	}

	// Record the frequency of outgoing BSM 
	private void sampleEncodedBSM() {
	    if(last_outgoing_sample_time == 0) {
            last_outgoing_sample_time = connectedNode_.getCurrentTime().toSeconds();
        }
        outgoing_bsm_counter++;
        if(connectedNode_.getCurrentTime().toSeconds() - last_outgoing_sample_time >= sample_window) {
            double freq = outgoing_bsm_counter / sample_window;
            log_.info("BSM", String.format("Outgoing BSM is encoded and published in %.02f Hz", freq));
            outgoing_bsm_counter = 0;
            last_outgoing_sample_time = connectedNode_.getCurrentTime().toSeconds();
        }
	}
	
	// Record the number of incoming BSM
	private void sampleDecodedBSM() {
	    if(last_incoming_sample_time == 0) {
            last_incoming_sample_time = connectedNode_.getCurrentTime().toSeconds();
        }
        incoming_bsm_counter++;
        if(connectedNode_.getCurrentTime().toSeconds() - last_incoming_sample_time >= sample_window) {
            log_.info("BSM", "There are " +  incoming_bsm_counter + " incoming BSMs decoded in past " + sample_window + " seconds");
            incoming_bsm_counter = 0;
            last_incoming_sample_time = connectedNode_.getCurrentTime().toSeconds(); 
        }
	}
	
	private void sendMessage(MessageContainer<?> message) {
	    if(message != null) {
	        switch (message.getType()) {
	        case "BSM":
	            bsmPub_.publish((BSM) message.getMessage());
	            sampleDecodedBSM();
	            break;
	        case "ByteArray":
	            outboundPub_.publish((ByteArray) message.getMessage());
	            break;
	        default:
	            log_.warn("Cannot find correct publisher for " + message.getType());
	        }
	    }
	}
	/***
	 * Handles unhandled exceptions and reports to SystemAlert topic, and log the alert.
	 * @param e The exception to handle
	 */
	@Override
	protected void handleException(Throwable e) {
		String msg = "Uncaught exception in " + connectedNode_.getName() + " caught by handleException";
		publishSystemAlert(AlertSeverity.FATAL, msg, e);
		connectedNode_.shutdown();
	}
}
