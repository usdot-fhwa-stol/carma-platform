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

	//Log for this node
	Log log = null;
	
	//Connected Node
	ConnectedNode connectedNode = null;
	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("message_consumer");
	}

	@Override
	public void onSaxtonStart(final ConnectedNode connectedNode) {
		
		//initialize connectedNode and log
		this.connectedNode = connectedNode;
		this.log = connectedNode.getLog();
		
		//initialize alert sub, pub
		alertPub = connectedNode.newPublisher("system_alert", SystemAlert._TYPE);
		alertSub = connectedNode.newSubscriber("system_alert", SystemAlert._TYPE);
		alertSub.addMessageListener(new MessageListener<SystemAlert>() {
			@Override
			public void onNewMessage(SystemAlert message) {
				try {
					if(message.getType() == SystemAlert.FATAL || message.getType() == SystemAlert.SHUTDOWN) {
						connectedNode.shutdown();
					}
					if(message.getType() == SystemAlert.DRIVERS_READY) {
						driversReady = true;
					}
				} catch (Exception e) {
					handleException(e);
				}
			}
		});
		
		//Use cav_srvs.GetDriversWithCapabilities.
		try {
			while(getDriversWithCapabilitiesClient == null) {
				if(driversReady) {
					getDriversWithCapabilitiesClient = this.waitForService("get_drivers_with_capabilities", GetDriversWithCapabilities._TYPE, connectedNode, 5000);
					if(getDriversWithCapabilitiesClient == null) {
						log.warn(connectedNode.getName() + " Node could not find service get_drivers_with_capabilities and is keeping trying...");
					}
				}
				Thread.sleep(1000);
			}	
		} catch (Exception e) {
			handleException(e);
		}
		
		
		
		//Subscribers
		hostBsmSub = connectedNode.newSubscriber("/saxton_cav/guidance/bsm", BSM._TYPE);
		hostBsmSub.addMessageListener(new MessageListener<BSM>() {
			@Override
			public void onNewMessage(BSM bsm) {
				try {
					if(outboundPub != null && driversReady) {
						log.info("MessageConsumer received BSM. Calling factory to encode data...");
						ByteArray byteArray = outboundPub.newMessage();
						BSMFactory.encode(bsm, byteArray, log);
						log.info("MessageConsumer finished encoding BSM and is going to publish...");
						outboundPub.publish(byteArray);
					}
				} catch (IllegalArgumentException ex) {
					log.info("Invalid BSM is not published");
				} catch (Exception e) {
					handleException(e);
				}
			}
		});

		// This CancellableLoop will be canceled automatically when the node shuts down.
		connectedNode.executeCancellableLoop(new CancellableLoop() {
			private int sequenceNumber;
			@Override
			protected void setup() { sequenceNumber = 0; } //setup
			@Override
			protected void loop() throws InterruptedException {
				if(outboundPub == null && driversReady) {
					//Setup request and make service call with synchronizer to GetDriversWithCapabilitiesResponse to find comms driver
					try {
						GetDriversWithCapabilitiesRequest request = getDriversWithCapabilitiesClient.newMessage();
						request.setCapabilities(Arrays.asList("inbound_binary_msg", "outbound_binary_msg"));
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
						outboundPub = connectedNode.newPublisher(responseCapabilities_comms.get(1), ByteArray._TYPE);
					} catch(Exception e) {
						handleException(e);
					}
				}
				sequenceNumber++;
				Thread.sleep(1000);
			}//loop
		});//executeCancellableLoop
	}// onStart

	@Override
	protected void handleException(Exception e) {
		log.error(connectedNode.getName() + "throws an exception and is about to shutdown...", e);
		connectedNode.shutdown();
	}
}
