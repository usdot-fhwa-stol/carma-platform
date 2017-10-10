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

package gov.dot.fhwa.saxton.carma.message;

import cav_msgs.*;
import cav_srvs.*;
import gov.dot.fhwa.saxton.carma.rosutils.SaxtonBaseNode;
import gov.dot.fhwa.saxton.carma.rosutils.RosServiceSynchronizer;

import java.nio.ByteOrder;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.apache.commons.logging.Log;
import org.jboss.netty.buffer.ChannelBuffer;
import org.jboss.netty.buffer.ChannelBuffers;
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

	private boolean driversReady = false;

	// Publisher
	// Some existed Pubs are not working. Disable them to focus on BSM.
	Publisher<SystemAlert> alertPub;
	Publisher<ByteArray> outboundPub;
	// TODO uncomment when messages are defined and BSM is ready.
	// protected Publisher<cav_msgs.BSM> bsmPub;
	// protected Publisher<cav_msgs.MobilityAck> mobilityAckPub;
	// protected Publisher<cav_msgs.MobilityGreeting> mobilityGreetingPub;
	// protected Publisher<cav_msgs.MobilityIntro> mobilityIntroPub;
	// protected Publisher<cav_msgs.MobilityNack> mobilityNAckPub;
	// protected Publisher<cav_msgs.MobilityPlan> mobilityPlanPub;
	// protected Publisher<cav_msgs.Map> mapPub;
	// protected Publisher<cav_msgs.Spat> spatPub;
	// protected Publisher<cav_msgs.Tim> timPub;

	// Subscribers
	// Some existed Subs are not working. Disable them to focus on BSM.
	Subscriber<SystemAlert> alertSub;
	Subscriber<BSM> hostBsmSub;
	// TODO uncomment when messages are defined
	// protected Subscriber<cav_msgs.MobilityAck> mobilityAckOutboundSub;
	// protected Subscriber<cav_msgs.MobilityGreeting> mobilityGreetingOutboundSub;
	// protected Subscriber<cav_msgs.MobilityIntro> mobilityIntroOutboundSub;
	// protected Subscriber<cav_msgs.MobilityNack> mobilityNAckOutboundSub;
	// protected Subscriber<cav_msgs.MobilityPlan> mobilityPlanOutboundSub;

	// Used Services
	ServiceClient<GetDriversWithCapabilitiesRequest, GetDriversWithCapabilitiesResponse> getDriversWithCapabilitiesClient;
	List<String> responseCapabilities_comms = new ArrayList();

	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("message_consumer");
	}

	@Override
	public void onSaxtonStart(final ConnectedNode connectedNode) {
		
		//System info: log, sub, pub
		final Log log = connectedNode.getLog();
		alertPub = connectedNode.newPublisher("system_alert", SystemAlert._TYPE);
		alertSub = connectedNode.newSubscriber("system_alert", SystemAlert._TYPE);
		alertSub.addMessageListener(new MessageListener<SystemAlert>() {
			@Override
			public void onNewMessage(SystemAlert message) {
				String messageTypeFullDescription = "Unknown system status";
				switch (message.getType()) {
					case SystemAlert.NOT_READY:
						driversReady = false;
						messageTypeFullDescription = "System not ready alert and will not publish";
						break;
					case SystemAlert.DRIVERS_READY:
						driversReady = true;
						messageTypeFullDescription = "System ready alert and is beginning to publish";
						break;
					case SystemAlert.SHUTDOWN:
						if(driversReady) {
							log.info("MessageConsumer: Received SHUTDOWN Message. Message node is shutting down...");
							connectedNode.shutdown();
						}
						break;
					default:
						if(driversReady) messageTypeFullDescription = "Unknown system alert type but system is ready";
						else messageTypeFullDescription = "Unknown system alert type and system is not ready";
				}
				log.info("message_consumer heard: " + message.getDescription() + "; " + messageTypeFullDescription);
			}
		});
		
		//Use cav_srvs.GetDriversWithCapabilities,
		getDriversWithCapabilitiesClient = this.waitForService("/saxton_cav/interface_manager/get_drivers_with_capabilities", GetDriversWithCapabilities._TYPE, connectedNode, 5000);
		if(getDriversWithCapabilitiesClient == null) {
			log.warn(connectedNode.getName() + " Node could not find service get_drivers_with_capabilities");
		}
		
		//Setup request and make service call with synchronizer to GetDriversWithCapabilitiesResponse to find comms driver
		try {
			GetDriversWithCapabilitiesRequest request = getDriversWithCapabilitiesClient.newMessage();
			request.setCapabilities(Arrays.asList("inbound_binary_msg", "outbound_binary_msg"));
			while(!driversReady) Thread.sleep(3000);
			RosServiceSynchronizer.callSync(getDriversWithCapabilitiesClient, request, new ServiceResponseListener<GetDriversWithCapabilitiesResponse>() {
				@Override
				public void onSuccess(GetDriversWithCapabilitiesResponse response) {
					responseCapabilities_comms = response.getDriverData();
					log.info("MessageConsumer GetDriversWithCapabilitiesResponse: " + responseCapabilities_comms);
				}
				@Override
				public void onFailure(RemoteException e) {
					throw new RosRuntimeException(e);
				}
			});
		} catch(Exception e) {
			log.error("Excepetion trapped while request driver capabilities");
			e.printStackTrace();
		}
		
		//Publishers
		outboundPub = connectedNode.newPublisher(responseCapabilities_comms.get(1), ByteArray._TYPE);
		
		//Subscribers
		hostBsmSub = connectedNode.newSubscriber("/saxton_cav/guidance/bsm", BSM._TYPE);
		hostBsmSub.addMessageListener(new MessageListener<BSM>() {
			@Override
			public void onNewMessage(BSM bsm) {
				log.info("MessageConsumer received BSM. Publishing it as ByteArray message...");
				ByteArray byteArray = outboundPub.newMessage();
				BSMFactory.encode(bsm, byteArray);
				outboundPub.publish(byteArray);
			}
		});		
		
    // Fake Pubs and Subs TODO: Remove!!!
    // The following are two example pub/subs for connecting to the mock arada driver using the launch file.
    // They should be removed as this process should be handled through the interface manager instead
    
//    final Subscriber<cav_msgs.ByteArray> recvSub = connectedNode.newSubscriber("/saxton_cav/drivers/arada_application/comms/recv", ByteArray._TYPE);
//    recvSub.addMessageListener(new MessageListener<ByteArray>() {
//      @Override public void onNewMessage(ByteArray byteArray) {
//        switch (byteArray.getMessageType()) {
//          case "BSM":
//            log.info("MessageConsumer received ByteArray of type BSM. Publishing BSM message");
//            bsmPub.publish(bsmPub.newMessage());
//            break;
//          case "MobilityAck":
//            log.info("MessageConsumer received ByteArray of type MobilityAck. Publishing MobilityAck message");
//            mobilityAckPub.publish(mobilityAckPub.newMessage());
//            break;
//          case "MobilityGreeting":
//            log.info("MessageConsumer received ByteArray of type MobilityGreeting. Publishing MobilityGreeting message");
//            mobilityGreetingPub.publish(mobilityGreetingPub.newMessage());
//            break;
//          case "MobilityIntro":
//            log.info("MessageConsumer received ByteArray of type MobilityIntro. Publishing MobilityIntro message");
//            mobilityIntroPub.publish(mobilityIntroPub.newMessage());
//            break;
//          case "MobilityNack":
//            log.info("MessageConsumer received ByteArray of type MobilityNack. Publishing MobilityNack message");
//            mobilityNAckPub.publish(mobilityNAckPub.newMessage());
//            break;
//        TODO uncomment when messages are defined
//        case "MobilityPlan":
//          log.info("MessageConsumer received ByteArray of type MobilityPlan. Publishing MobilityPlan message");
//          mobilityPlanPub.publish(mobilityPlanPub.newMessage());
//          break;
//          default:
//            log.info("MessageConsumer received ByteArray of type Unknown. Publishing as example BSM message");
//            bsmPub.publish(bsmPub.newMessage());
//        }
//      }
//    });
		
		
//    bsmPub = connectedNode.newPublisher("bsm", cav_msgs.BSM._TYPE);
//    mobilityAckPub = connectedNode.newPublisher("mobility_ack_recv", cav_msgs.MobilityAck._TYPE);
//    mobilityGreetingPub = connectedNode.newPublisher("mobility_greeting_recv", cav_msgs.MobilityGreeting._TYPE);
//    mobilityIntroPub = connectedNode.newPublisher("mobility_intro_recv", cav_msgs.MobilityIntro._TYPE);
//    mobilityNAckPub = connectedNode.newPublisher("mobility_nack_recv", cav_msgs.MobilityNack._TYPE);
//    TODO uncomment when messages are defined
//    mobilityPlanPub = connectedNode.newPublisher("mobility_plan_recv", cav_msgs.MobilityPlan._TYPE);
//    mapPub = connectedNode.newPublisher("map", cav_msgs.Map._TYPE);
//    spatPub = connectedNode.newPublisher("spat", cav_msgs.Spat._TYPE);
//    timPub = connectedNode.newPublisher("tim", cav_msgs.Tim._TYPE);


		

//    mobilityAckOutboundSub = connectedNode.newSubscriber("mobility_ack_outbound", cav_msgs.MobilityAck._TYPE);
//    mobilityAckOutboundSub.addMessageListener(new MessageListener<MobilityAck>() {
//      @Override public void onNewMessage(MobilityAck mobilityAck) {
//        if (systemReady) {
//          log.info("MessageConsumer received BSM outbound. Publishing as ByteArray message");
//          ByteArray byteArray = outboundPub.newMessage();
//          byteArray.setMessageType("MobilityAck"); // Not sure if this is correct type use but will help validate messaging
//          outboundPub.publish(byteArray);
//        }
//      }
//    });
//
//    mobilityGreetingOutboundSub = connectedNode.newSubscriber("mobility_greeting_outbound", cav_msgs.MobilityGreeting._TYPE);
//    mobilityGreetingOutboundSub.addMessageListener(new MessageListener<MobilityGreeting>() {
//      @Override public void onNewMessage(MobilityGreeting mobilityGreeting) {
//        if (systemReady) {
//          log.info("MessageConsumer received BSM outbound. Publishing as ByteArray message");
//          ByteArray byteArray = outboundPub.newMessage();
//          byteArray.setMessageType("MobilityGreeting"); // Not sure if this is correct type use but will help validate messaging
//          outboundPub.publish(byteArray);
//        }
//      }
//    });
//
//    mobilityIntroOutboundSub = connectedNode.newSubscriber("mobility_intro_outbound", cav_msgs.MobilityIntro._TYPE);
//    mobilityIntroOutboundSub.addMessageListener(new MessageListener<MobilityIntro>() {
//      @Override public void onNewMessage(MobilityIntro mobilityIntro) {
//        if (systemReady) {
//          log.info("MessageConsumer received BSM outbound. Publishing as ByteArray message");
//          ByteArray byteArray = outboundPub.newMessage();
//          byteArray.setMessageType("MobilityIntro"); // Not sure if this is correct type use but will help validate messaging
//          outboundPub.publish(byteArray);
//        }
//      }
//    });
//
//    mobilityNAckOutboundSub = connectedNode.newSubscriber("mobility_nack_outbound", cav_msgs.MobilityNack._TYPE);
//    mobilityNAckOutboundSub.addMessageListener(new MessageListener<MobilityNack>() {
//      @Override public void onNewMessage(MobilityNack mobilityNack) {
//        if (systemReady ) {
//          log.info("MessageConsumer received BSM outbound. Publishing as ByteArray message");
//          ByteArray byteArray = outboundPub.newMessage();
//          byteArray.setMessageType("MobilityNack"); // Not sure if this is correct type use but will help validate messaging
//          outboundPub.publish(byteArray);
//        }
//      }
//    });
//    TODO Uncomment when messages are defined
//    mobilityPlanOutboundSub = connectedNode.newSubscriber("mobility_plan_outbound", cav_msgs.MobilityPlan._TYPE);
//    mobilityPlanOutboundSub.addMessageListener(new MessageListener<MobilityPlan>() {
//      @Override public void onNewMessage(MobilityPlan mobilityPlan) {
//        if (systemReady) {
//          log.info("MessageConsumer received MobilityPlan outbound. Publishing as ByteArray message");
//          ByteArray byteArray = outboundPub.newMessage();
//          byteArray.setMessageType("MobilityPlan"); // Not sure if this is correct type use but will help validate messaging
//          outboundPub.publish(byteArray);
//        }
//      }
//    });

		// This CancellableLoop will be canceled automatically when the node shuts down.
		connectedNode.executeCancellableLoop(new CancellableLoop() {
			private int sequenceNumber;
			@Override
			protected void setup() { sequenceNumber = 0; } //setup
			@Override
			protected void loop() throws InterruptedException {
				sequenceNumber++;
				Thread.sleep(1000);
			}//loop
		});//executeCancellableLoop
	}// onStart

	@Override
	protected void handleException(Exception e) { }
}// AbstractNodeMain
