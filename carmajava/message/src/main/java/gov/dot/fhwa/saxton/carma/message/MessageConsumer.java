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

package gov.dot.fhwa.saxton.carma.message;

import gov.dot.fhwa.saxton.carma.message.factory.DSRCMessageFactory;
import gov.dot.fhwa.saxton.carma.message.factory.IMessage;
import gov.dot.fhwa.saxton.carma.message.factory.MessageContainer;
import gov.dot.fhwa.saxton.carma.message.helper.MessageStatistic;
import gov.dot.fhwa.saxton.carma.rosutils.AlertSeverity;
import gov.dot.fhwa.saxton.carma.rosutils.SaxtonBaseNode;
import gov.dot.fhwa.saxton.carma.rosutils.SaxtonLogger;

import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;

import org.ros.message.MessageListener;
import org.ros.node.parameter.ParameterTree;
import org.ros.node.topic.Subscriber;

import cav_msgs.ByteArray;
import cav_msgs.MobilityOperation;
import cav_msgs.MobilityPath;
import cav_msgs.MobilityRequest;
import cav_msgs.MobilityResponse;
import cav_msgs.SystemAlert;

import j2735_msgs.BSM;
import j2735_msgs.SPAT;
import j2735_msgs.MapData;

import org.ros.concurrent.CancellableLoop;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;

import org.ros.exception.RosRuntimeException;

/*
 * The Message package is part of the Vehicle Environment package.
 * It processes all V2V and V2I messages coming from Drivers.Comms ROS node.
 * Command line test: rosrun carma message gov.dot.fhwa.saxton.carma.message.MessageConsumer
 * rostopic pub /system_alert cav_msgs/SystemAlert '{type: 5, description: hello}'
 * rosparam set /interface_mgr/driver_wait_time 10
 * rosrun carma interfacemgr gov.dot.fhwa.saxton.carma.interfacemgr.InterfaceMgr
 * rostopic pub /carma/drivers/arada_application/comms/recv cav_msgs/ByteArray '{messageType: "BSM"}'
 * rostopic pub /host_bsm j2735_msgs/BSM '{}'
 */

public class MessageConsumer extends SaxtonBaseNode {

	protected boolean driversReady = false;
	protected volatile boolean shutdownInitialized = false;

	// Publishers
	protected Publisher<ByteArray> outboundPub_; //outgoing byte array after encode
	protected Publisher<BSM> bsmPub_; //incoming BSM after decoded
	protected Publisher<MobilityRequest> mobilityReqPub_; //incoming mobility request message after decoded
	protected Publisher<MobilityPath> mobilityPathPub_; //incoming mobility path message after decoded
	protected Publisher<MobilityResponse> mobilityResponsePub_; //incoming mobility response message after decoded
	protected Publisher<MobilityOperation> mobilityOperationPub_; //incoming mobility operation message after decoded
	protected Publisher<MapData> mapPub_; //incoming MAP message after decoded
	protected Publisher<SPAT> spatPub_; //incoming SPAT message after decoded
	
	// Subscribers
	protected Subscriber<SystemAlert> alertSub_;
	protected Subscriber<ByteArray> inboundSub_; //incoming byte array, need to decode
	protected Subscriber<BSM> bsmSub_; //outgoing plain BSM
	protected Subscriber<MobilityRequest> mobilityReqSub_; //outgoing plain mobility request message
	protected Subscriber<MobilityPath> mobilityPathSub_; //outgoing plain mobility path message
	protected Subscriber<MobilityResponse> mobilityResponseSub_; //outgoing plain mobility response message
	protected Subscriber<MobilityOperation> mobilityOperationSub_; //outgoing plain mobility operation message
	
	// Log for this node
    protected SaxtonLogger log_ = null;
	
	// Connected Node
	protected ConnectedNode connectedNode_ = null;
	
	// Recoding message frequency
	protected MessageStatistic messageCounters = null;
	
	// Messages to be encoded
	protected BlockingQueue<MessageContainer> dsrcMessageQueue = new LinkedBlockingQueue<>();

	// Configure parameters
    protected boolean publishOutboundBsm_ = true;
    protected boolean publishOutboundMobilityRequest_ = true;
    protected boolean publishOutboundMobilityPath_ = true;
    protected boolean publishOutboundMobilityResponse_ = true;
    protected boolean publishOutboundMobilityOperation_ = true;
    
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
            publishOutboundBsm_ = param.getBoolean("~/publish_outbound_bsm", true);
            publishOutboundMobilityRequest_ = param.getBoolean("~/publish_outbound_mobility_request", true);
            publishOutboundMobilityPath_ = param.getBoolean("~/publish_outbound_mobility_path", true);
            publishOutboundMobilityResponse_ = param.getBoolean("~/publish_outbound_mobility_response", true);
            publishOutboundMobilityOperation_ = param.getBoolean("~/publish_outbound_mobility_operation", true);
        }catch (Exception e) {
            log_.warn("STARTUP", "Error reading Message parameters. Using defaults.");
        }
        log_.debug("Read params to publish outbound: BSM = " + publishOutboundBsm_ + ", REQUEST = " + publishOutboundMobilityRequest_);
        log_.debug("Read params to publish outbound: PATH = " + publishOutboundMobilityPath_ + ", RESPONSE = " + publishOutboundMobilityResponse_);
        log_.debug("Read params to publish outbound: OPERATION = " + publishOutboundMobilityOperation_);

        //initialize message statistic
		messageCounters = new MessageStatistic(connectedNode_, log_);
		
		//initialize alert sub, pub
		alertSub_ = this.connectedNode_.newSubscriber("system_alert", SystemAlert._TYPE);
		if(alertSub_ != null) {
		    alertSub_.addMessageListener(new MessageListener<SystemAlert>() {
                @Override
                public void onNewMessage(SystemAlert message) {
                    if(message.getType() == SystemAlert.FATAL || message.getType() == SystemAlert.SHUTDOWN) {
                        if(!shutdownInitialized) {
                            shutdownInitialized = true;
                            connectedNode_.shutdown();
                        }
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
		
		//initialize Pubs
		bsmPub_ = connectedNode_.newPublisher("incoming_j2735_bsm", BSM._TYPE);
		outboundPub_ = connectedNode_.newPublisher("outbound_binary_msg", ByteArray._TYPE);
		mobilityReqPub_ = connectedNode_.newPublisher("incoming_mobility_request", MobilityRequest._TYPE);
		mobilityPathPub_ = connectedNode_.newPublisher("incoming_mobility_path", MobilityPath._TYPE);
		mobilityResponsePub_ = connectedNode_.newPublisher("incoming_mobility_response", MobilityResponse._TYPE);
		mobilityOperationPub_ = connectedNode_.newPublisher("incoming_mobility_operation", MobilityOperation._TYPE);
		mapPub_ = connectedNode_.newPublisher("incoming_j2735_map", MapData._TYPE);
		spatPub_ = connectedNode_.newPublisher("incoming_j2735_spat", SPAT._TYPE);
		if(bsmPub_ == null || outboundPub_ == null || mobilityReqPub_ == null ||
		   mobilityPathPub_ == null || mobilityResponsePub_ == null ||
		   mobilityOperationPub_ == null || mapPub_ == null || spatPub_ == null) {
		    log_.error("Cannot initialize necessary publishers.");
		    handleException(new RosRuntimeException("Cannot initialize necessary publishers."));
		}
		
		//register new message counters
		messageCounters.registerEntry("BSM");
		messageCounters.registerEntry("MobilityRequest");
		messageCounters.registerEntry("MobilityPath");
		messageCounters.registerEntry("MobilityResponse");
		messageCounters.registerEntry("MobilityOperation");
		messageCounters.registerEntry("MAP");
		messageCounters.registerEntry("SPAT");
		
		//initialize Subs
		bsmSub_ = connectedNode_.newSubscriber("outgoing_j2735_bsm", BSM._TYPE);
		inboundSub_ = connectedNode_.newSubscriber("inbound_binary_msg", ByteArray._TYPE);
		mobilityReqSub_ = connectedNode_.newSubscriber("outgoing_mobility_request", MobilityRequest._TYPE);
		mobilityPathSub_ = connectedNode_.newSubscriber("outgoing_mobility_path", MobilityPath._TYPE);
		mobilityResponseSub_ = connectedNode_.newSubscriber("outgoing_mobility_response", MobilityResponse._TYPE);
		mobilityOperationSub_ = connectedNode_.newSubscriber("outgoing_mobility_operation", MobilityOperation._TYPE);
		if(bsmSub_ == null || inboundSub_ == null || mobilityReqSub_ == null ||
		   mobilityPathSub_ == null || mobilityResponseSub_ == null || mobilityOperationSub_ == null) {
		    log_.error("Cannot initialize necessary subscribers.");
		    handleException(new RosRuntimeException("Cannot initialize necessary subscribers."));
		}
        bsmSub_.addMessageListener((bsm) -> dsrcMessageQueue.add(new MessageContainer("BSM", bsm)));
        mobilityReqSub_.addMessageListener((req) -> dsrcMessageQueue.add(new MessageContainer("MobilityRequest", req)));
        mobilityPathSub_.addMessageListener((path) -> dsrcMessageQueue.add(new MessageContainer("MobilityPath", path)));
        mobilityResponseSub_.addMessageListener((response) -> dsrcMessageQueue.add(new MessageContainer("MobilityResponse", response)));
        mobilityOperationSub_.addMessageListener((op) -> dsrcMessageQueue.add(new MessageContainer("MobilityOperation", op)));
        inboundSub_.addMessageListener((msg) -> {
		    messageCounters.onMessageReceiving(msg.getMessageType());
		    IMessage<?> message = DSRCMessageFactory.getMessage(msg.getMessageType(), connectedNode_, log_, connectedNode_.getTopicMessageFactory());
		    if(message != null) {
		        MessageContainer decodedMessage = message.decode(msg);
	            if(decodedMessage.getMessage() != null) {
	                switch (decodedMessage.getType()) {
	                case "BSM":
	                    bsmPub_.publish((BSM) decodedMessage.getMessage());
	                    break;
	                case "MobilityRequest":
	                    mobilityReqPub_.publish((MobilityRequest) decodedMessage.getMessage());
	                    log_.debug("V2V", "Received & decoded MobilityRequest, plan ID = " +
	                                ((MobilityRequest) decodedMessage.getMessage()).getHeader().getPlanId());
	                    break;
	                case "MobilityPath":
	                    mobilityPathPub_.publish((MobilityPath) decodedMessage.getMessage());
	                    log_.debug("V2V", "Received & decoded MobilityPath, plan ID = " +
	                                ((MobilityPath) decodedMessage.getMessage()).getHeader().getPlanId());
	                    break;
	                case "MobilityResponse":
	                    mobilityResponsePub_.publish((MobilityResponse) decodedMessage.getMessage());
	                    log_.debug("V2V", "Received & decoded MobilityResponse, plan ID = " +
								((MobilityResponse) decodedMessage.getMessage()).getHeader().getPlanId());
						break;
	                case "MobilityOperation":
	                    mobilityOperationPub_.publish((MobilityOperation) decodedMessage.getMessage());
	                    log_.debug("V2V", "Received & decoded MobilityOperation, plan ID = " +
								((MobilityOperation) decodedMessage.getMessage()).getHeader().getPlanId());
						break;
	                case "MAP":
	                    MapData map = (MapData) decodedMessage.getMessage();
	                    map.getHeader().setStamp(connectedNode_.getCurrentTime());
	                    mapPub_.publish(map);
                            break;
	                case "SPAT":
	                    spatPub_.publish((SPAT) decodedMessage.getMessage());
                            break;
	                default:
	                    log_.warn("Cannot find correct publisher for " + decodedMessage.getType());
	                }
	            }
		    }
		});
		
		// This CancellableLoop will be canceled automatically when the node shuts down.
		connectedNode_.executeCancellableLoop(new CancellableLoop() {
			@Override
			protected void loop() throws InterruptedException {
			    MessageContainer outgoingMessage = dsrcMessageQueue.take();
			    String mtype = outgoingMessage.getType();
			    if((mtype.equals("BSM")             && publishOutboundBsm_) ||
				   (mtype.equals("MobilityRequest") && publishOutboundMobilityRequest_) ||
				   (mtype.equals("MobilityPath") && publishOutboundMobilityPath_) ||
				   (mtype.equals("MobilityResponse") && publishOutboundMobilityResponse_) ||
				   (mtype.equals("MobilityOperation") && publishOutboundMobilityOperation_)) {
			        IMessage<?> message = DSRCMessageFactory.getMessage(outgoingMessage.getType(), connectedNode_, log_, connectedNode_.getTopicMessageFactory());
                    if(message != null) {
                        log_.debug("Found message factory on type " + outgoingMessage.getType());
                        MessageContainer encodedMessage = message.encode(outgoingMessage.getMessage());
                        if(encodedMessage.getMessage() != null) {
                            log_.debug("We encode " + outgoingMessage.getType());
                            messageCounters.onMessageSending(((ByteArray) encodedMessage.getMessage()).getMessageType());
                            outboundPub_.publish((ByteArray) encodedMessage.getMessage());
                        } else {
                            log_.warn("We failed to encode " + outgoingMessage.getType());
                        }
                    }
			        
			    }
			}
		});
		
	}
	
	@Override
	protected void handleException(Throwable e) {
		String msg = "Uncaught exception in " + connectedNode_.getName() + " caught by handleException";
		publishSystemAlert(AlertSeverity.FATAL, msg, e);
		if(!this.shutdownInitialized) {
		    shutdownInitialized = true;
		    connectedNode_.shutdown();
		}
	}
}
