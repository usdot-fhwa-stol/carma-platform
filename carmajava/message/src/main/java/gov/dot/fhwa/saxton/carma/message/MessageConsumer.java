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
import gov.dot.fhwa.saxton.carma.message.factory.DSRCMessageFactory;
import gov.dot.fhwa.saxton.carma.message.factory.IMessage;
import gov.dot.fhwa.saxton.carma.message.factory.MessageContainer;
import gov.dot.fhwa.saxton.carma.message.helper.MessageStatistic;
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
import org.ros.node.parameter.ParameterTree;
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
	protected Publisher<ByteArray> outboundPub_; //outgoing byte array after encode
	protected Publisher<BSM> bsmPub_; //incoming BSM after decoded
	protected Publisher<MobilityIntro> mobilityIntroPub_; //incoming mobility introduction message after decoded
	protected Publisher<MobilityAck> mobilityAckPub_; //incoming mobility ack message after decoded

	// Subscribers
	protected Subscriber<SystemAlert> alertSub_;
	protected Subscriber<ByteArray> inboundSub_; //incoming byte array, need to decode
	protected Subscriber<BSM> bsmSub_; //outgoing BSM, need to encode
	protected Subscriber<MobilityIntro> mobilityIntroSub_; //outgoing mobility introduction message, need to encode
	protected Subscriber<MobilityAck> mobilityAckSub_; //outgoing mobility ack message, need to encode

	// Used Services
	protected ServiceClient<GetDriversWithCapabilitiesRequest, GetDriversWithCapabilitiesResponse> getDriversWithCapabilitiesClient_;
	
	// Log for this node
    protected SaxtonLogger log_ = null;
	
	// Connected Node
	protected ConnectedNode connectedNode_ = null;
	
	// Recoding message frequency
	protected MessageStatistic messageCounters = null;
	
	// Messages to be encoded
	protected BlockingQueue<MessageContainer> dsrcMessageQueue = new LinkedBlockingQueue<>();

	// Config parameters
    protected boolean publishOutboundBsm_ = true;
    protected boolean publishOutboundMobilityIntro_ = true;
    protected boolean publishOutboundMobilityAck_ = true;
    protected boolean publishOutboundMobilityGreeting_ = true;
    protected boolean publishOutboundMobilityPlan_ = true;
	
	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("message_consumer");
	}

	@Override
	public void onSaxtonStart(final ConnectedNode connectedNode) {
		
		//initialize connectedNode and log
		this.connectedNode_ = connectedNode;
		this.log_ = new SaxtonLogger(MessageConsumer.class.getSimpleName(), this.connectedNode_.getLog());

		//get config params
        try {
            ParameterTree param = connectedNode.getParameterTree();
            publishOutboundBsm_ = param.getBoolean("~/publish_outbound_bsm");
            publishOutboundMobilityIntro_ = param.getBoolean("~/publish_outbound_mobility_intro");
            publishOutboundMobilityGreeting_ = param.getBoolean("~/publish_outbound_mobility_greeting");
            publishOutboundMobilityAck_ = param.getBoolean("~/publish_outbound_mobility_ack");
            publishOutboundMobilityPlan_ = param.getBoolean("~/publish_outbound_mobility_plan");
            log_.info("Read params to publish outbound: BSM = " + publishOutboundBsm_ + ", Mob intro = " + publishOutboundMobilityIntro_
                    + ", Mob greeting = " + publishOutboundMobilityGreeting_ + ", Mob ack = " + publishOutboundMobilityAck_
                    + ", Mob plan = " + publishOutboundMobilityPlan_);
        }catch (Exception e) {
            log_.warn("STARTUP", "Error reading Message parameters. Using defaults.");
        }

        //initialize message statistic
		messageCounters = new MessageStatistic(connectedNode_, log_);
		
		//initialize alert sub, pub
		alertSub_ = this.connectedNode_.newSubscriber("system_alert", SystemAlert._TYPE);
		if(alertSub_ != null) {
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
		} else {
		    log_.error("Cannot initialize system_alert subscriber!");
		    handleException(new RosRuntimeException("MessageConsumer cannot initialize system_alert sub"));
		}
		
		//wait for driversReady
        while(!driversReady) {
            try {
                Thread.sleep(1000);
            } catch(InterruptedException e) {
                log_.warn("Wait for driversReady has been interrupted.");
                Thread.interrupted();
            }
        }
		
		//Use cav_srvs.GetDriversWithCapabilities and wait for driversReady signal
		getDriversWithCapabilitiesClient_ = this.waitForService("get_drivers_with_capabilities", GetDriversWithCapabilities._TYPE, connectedNode, 10000);
		if(getDriversWithCapabilitiesClient_ == null) {
			log_.error("Cannot find service get_drivers_with_capabilities");
            handleException(new ServiceNotFoundException("get_drivers_with_capabilities is not found!"));
		}
		
		//Make service calls and validate drivers information
		GetDriversWithCapabilitiesRequest request = getDriversWithCapabilitiesClient_.newMessage();
		List<String> capabilities = Arrays.asList("inbound_binary_msg", "outbound_binary_msg");
		request.setCapabilities(capabilities);
		final List<GetDriversWithCapabilitiesResponse> driver_response = new ArrayList<>();
		try {
            RosServiceSynchronizer.callSync(getDriversWithCapabilitiesClient_, request, new ServiceResponseListener<GetDriversWithCapabilitiesResponse>() {         	
            	@Override
            	public void onSuccess(GetDriversWithCapabilitiesResponse response) {
            	    driver_response.add(response);
            		log_.info("GetDriversWithCapabilitiesResponse: " + driver_response.get(0).getDriverData());
            	}
            	@Override
            	public void onFailure(RemoteException e) {
            		throw new RosRuntimeException(e);
            	}
            });
        } catch (Exception e1) {
            log_.error("GetDriversWithCapabilities call failed.");
            handleException(e1);
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
			log_.error("Unable to find suitable dsrc drivers!");
            handleException(new RosRuntimeException("Cannot find suitable DSRC drivers."));
		}
		
		//initialize Pubs
		bsmPub_ = connectedNode_.newPublisher("incoming_bsm", BSM._TYPE);
		outboundPub_ = connectedNode_.newPublisher(J2735_outbound_binary_msg, ByteArray._TYPE);
		mobilityIntroPub_ = connectedNode_.newPublisher("incoming_intro", MobilityIntro._TYPE);
		mobilityAckPub_ = connectedNode_.newPublisher("incoming_ack", MobilityAck._TYPE);
		if(bsmPub_ == null || outboundPub_ == null || mobilityIntroPub_ == null || mobilityAckPub_ == null) {
		    log_.error("Cannot initialize necessary publishers.");
		    handleException(new RosRuntimeException("Cannot initialize necessary publishers."));
		}
		
		//register new message counters
		messageCounters.registerEntry("BSM");
		
		//initialize Subs
		bsmSub_ = connectedNode_.newSubscriber("outgoing_bsm", BSM._TYPE);
		inboundSub_ = connectedNode_.newSubscriber(J2735_inbound_binary_msg, ByteArray._TYPE);
		mobilityIntroSub_ = connectedNode_.newSubscriber("mobility_intro_outbound", MobilityIntro._TYPE);
		mobilityAckSub_ = connectedNode_.newSubscriber("mobility_ack_outbound", MobilityAck._TYPE);
		if(bsmSub_ == null || inboundSub_ == null || mobilityIntroSub_ == null || mobilityAckSub_ == null) {
		    log_.error("Cannot initialize necessary subscribers.");
		    handleException(new RosRuntimeException("Cannot initialize necessary subscribers."));
		}
        bsmSub_.addMessageListener((bsm) -> dsrcMessageQueue.add(new MessageContainer("BSM", bsm)));
        mobilityIntroSub_.addMessageListener((intro) -> dsrcMessageQueue.add(new MessageContainer("MobilityIntro", intro)));
        mobilityAckSub_.addMessageListener((ack) -> dsrcMessageQueue.add(new MessageContainer("MobilityAck", ack)));
        inboundSub_.addMessageListener((msg) -> {
		    messageCounters.onMessageReceiving(msg.getMessageType());
		    IMessage<?> message = DSRCMessageFactory.getMessage(msg.getMessageType(), connectedNode_, log_, connectedNode_.getTopicMessageFactory());
		    MessageContainer decodedMessage = message.decode(msg);
		    if(decodedMessage.getMessage() != null) {
		        switch (decodedMessage.getType()) {
	            case "BSM":
	                if (publishOutboundBsm_) {
                        bsmPub_.publish((BSM) decodedMessage.getMessage());
                    }
	                break;
	            case "MobilityIntro":
	                if (publishOutboundMobilityIntro_) {
                        mobilityIntroPub_.publish((MobilityIntro) decodedMessage.getMessage());
                    }
	                break;
	            case "MobilityAck":
	                if (publishOutboundMobilityAck_) {
                        mobilityAckPub_.publish((MobilityAck) decodedMessage.getMessage());
                    }
	            default:
	                log_.warn("Cannot find correct publisher for " + decodedMessage.getType());
	            }
		    }
		});
		
		// This CancellableLoop will be canceled automatically when the node shuts down.
		connectedNode_.executeCancellableLoop(new CancellableLoop() {
			@Override
			protected void loop() throws InterruptedException {
			    MessageContainer outgoingMessage = dsrcMessageQueue.take();
				IMessage<?> message = DSRCMessageFactory.getMessage(outgoingMessage.getType(), connectedNode_, log_, connectedNode_.getTopicMessageFactory());
				MessageContainer encodedMessage = message.encode(outgoingMessage.getMessage());
				if(encodedMessage.getMessage() != null) {
				    log_.info("We encode " + outgoingMessage.getType());
				    messageCounters.onMessageSending(((ByteArray) encodedMessage.getMessage()).getMessageType());
	                outboundPub_.publish((ByteArray) encodedMessage.getMessage());
				} else {
				    log_.warn("We failed to encode " + outgoingMessage.getType());
				}
			}
		});
	}
	
	@Override
	protected void handleException(Throwable e) {
		String msg = "Uncaught exception in " + connectedNode_.getName() + " caught by handleException";
		publishSystemAlert(AlertSeverity.FATAL, msg, e);
		connectedNode_.shutdown();
	}
}
