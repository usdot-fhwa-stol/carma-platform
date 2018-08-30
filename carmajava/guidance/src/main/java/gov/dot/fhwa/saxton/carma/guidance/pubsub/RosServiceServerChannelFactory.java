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

import gov.dot.fhwa.saxton.carma.guidance.GuidanceExceptionHandler;
import gov.dot.fhwa.saxton.carma.rosutils.SaxtonBaseNode;
import org.ros.node.ConnectedNode;
import org.ros.node.service.ServiceServer;

/**
 * Concrete ROS implementation of the logic outlined in {@link IServiceServerChannelFactory}
 *
 * Uses an {@link ConnectedNode} instance to create {@link ServiceServer} instances for use by
 * {@link RosServiceServerChannel} instances and their children {@link RosServiceServer} instances
 */
public class RosServiceServerChannelFactory implements IServiceServerChannelFactory {
    protected final ConnectedNode conn;
    protected final GuidanceExceptionHandler exceptionHandler;

    /**
     * Constructor
     * 
     * @param conn The ROS java connected node instance which will provide connection to the ros network
     * @param exceptionHandler A guidance exception handler
     */
    public RosServiceServerChannelFactory(ConnectedNode conn, GuidanceExceptionHandler exceptionHandler) {
        this.conn = conn;
        this.exceptionHandler = exceptionHandler;
    }

    @Override public <T, S> IServiceServerChannel<T, S> newServiceServerChannel(String topic, String type, OnServiceRequestCallback<T,S> callback) { // TODO add topic not found exception
        
        return new RosServiceServerChannel <T, S> (
            conn.newServiceServer(topic, type,
                (T request, S response) -> {
                    try {
                        callback.onRequest(request, response);
                    } catch (Exception e) {
                      exceptionHandler.handleException(e);
                    }
                }
            )
        );
    }
}
