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

package gov.dot.fhwa.saxton.carma.guidance.pubsub;

import org.ros.message.MessageFactory;
import org.ros.node.NodeConfiguration;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceServer;

/**
 * Concrete ROS implementation of the logic outlined in {@link IServiceServerChannel}
 *
 * Shares access to a {@link ServiceServer} created by an {@link IServiceServerChannelFactory} between
 * any number of child {@link RosServiceServer} instances
 * @param <T> Type parameter for the service request message
 * @param <S> Type parameter for the service response message
 */
public class RosServiceServerChannel<T, S> implements IServiceServerChannel<T, S> {
    protected boolean open = true;
    protected ServiceServer<T, S> serviceServer;
    MessageFactory messageFactory = NodeConfiguration.newPrivate().getTopicMessageFactory();

    /**
     * Constructor
     * 
     * @param serviceServer The underlying ROS Java service server which will provide connection to the ros network
     */
    RosServiceServerChannel(ServiceServer<T, S> serviceServer) {
        this.serviceServer = serviceServer;
    }

    @Override public boolean isOpen() {
        return open;
    }

    @Override public void close() {
        serviceServer.shutdown();
        open = false;
    }
    

	@Override
	public IServiceServer<T, S> getServiceServer() {
		return new RosServiceServer<T,S>(this);
	}
}
