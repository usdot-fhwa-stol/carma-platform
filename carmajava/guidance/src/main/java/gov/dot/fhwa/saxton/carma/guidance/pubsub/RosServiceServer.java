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


import org.ros.node.service.ServiceClient;

/**
 * Concrete ROS implementation of the logic outlined in {@link IServiceServer}
 * 
 * @param <T> Type parameter for the request message type for the service
 * @param <S> Type parameter for the response message type for the service
 */
public class RosServiceServer<T, S> implements IServiceServer<T, S> {
    protected OnServiceRequestCallback<T, S> callback;

    /**
     * Constructor
     * 
     * @param callback The callback which will be triggered on a service request
     */
    protected RosServiceServer(OnServiceRequestCallback<T, S> callback) {
        this.callback = callback;
    }

    @Override
    public void setCallback(OnServiceRequestCallback<T, S> callback) {
        this.callback = callback;
    }

    @Override
    public OnServiceRequestCallback<T, S> getCallback() {
        return callback;
    }
}
