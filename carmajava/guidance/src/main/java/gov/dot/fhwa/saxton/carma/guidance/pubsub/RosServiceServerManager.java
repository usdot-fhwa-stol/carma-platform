/*
 * Copyright (C) 2018-2019 LEIDOS.
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
import gov.dot.fhwa.saxton.carma.guidance.util.ILogger;
import gov.dot.fhwa.saxton.carma.guidance.util.LoggerManager;

import java.util.Collections;
import java.util.HashMap;
import java.util.Map;

import org.ros.node.ConnectedNode;
import org.ros.node.service.ServiceServer;

/**
 * Concrete ROS implementation of the logic outlined in {@link IServiceServerManager}
 *
 * Uses an {@link ConnectedNode} instance to create {@link ServiceServer} instances
 * Services which already exist will have their callbacks updated.
 */
public class RosServiceServerManager implements IServiceServerManager {
    protected final ConnectedNode conn;
    protected final GuidanceExceptionHandler exceptionHandler;
    protected final ILogger log = LoggerManager.getLogger();
    protected Map<String, IServiceServer<?, ?>> serviceServers;
    protected Map<String, ServiceServer<?, ?>> rosServiceServers;

    /**
     * Constructor
     * 
     * @param conn The ROS java connected node instance which will provide connection to the ros network
     * @param exceptionHandler A guidance exception handler
     */
    public RosServiceServerManager(ConnectedNode conn, GuidanceExceptionHandler exceptionHandler) {
        this.conn = conn;
        this.exceptionHandler = exceptionHandler;
        serviceServers = Collections.synchronizedMap(new HashMap<String, IServiceServer<?, ?>>());
        rosServiceServers = Collections.synchronizedMap(new HashMap<String, ServiceServer<?, ?>>());
    }

    @SuppressWarnings("unchecked")
    @Override
    public <T, S> void createServiceServerForTopic(String topicUrl, String type,
            OnServiceRequestCallback<T, S> callback) {

        // Create or update callback wrapper
        synchronized (serviceServers) {
            if (serviceServers.containsKey(topicUrl)) {
                ((IServiceServer<T,S>) serviceServers.get(topicUrl)).setCallback(callback);
                log.warn("A second service server creation request was received for an already existing service server. If this was caused by a soft restart, ignore this message");
                return; // No need to create the ros service server if it already exists
            } else {
                serviceServers.put(topicUrl, new RosServiceServer<T,S>(callback));
            }
        }

        // Get callback wrapper
        IServiceServer<T,S> server = (IServiceServer<T,S>) serviceServers.get(topicUrl);

        // Create ros service server
        rosServiceServers.put(topicUrl, 
            conn.newServiceServer(topicUrl, type,
                (T request, S response) -> {
                    try {
                        server.getCallback().onRequest(request, response);
                    } catch (Exception e) {
                        exceptionHandler.handleException(e);
                    }
                }
            )
        );
    }
}
