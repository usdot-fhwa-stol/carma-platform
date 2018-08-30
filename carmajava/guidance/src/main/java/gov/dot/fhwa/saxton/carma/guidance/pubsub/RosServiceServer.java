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
 * Shares a {@link ServiceServer} between its coworkers (other RosServiceServer instances for the same
 * topic) and its parent {@link RosServiceServerChannel}.
 * 
 * @param <T> Type parameter for the request message type for the service
 * @param <S> Type parameter for the response message type for the service
 */
public class RosServiceServer<T, S> implements IServiceServer<T, S> {
    protected RosServiceServerChannel<T, S> parent;
    protected boolean open = true;

    /**
     * Constructor
     * 
     * @param parent The ros service channel which provides connection to the ros network
     */
    RosServiceServer(RosServiceServerChannel<T, S> parent) {
        this.parent = parent;
    }

    @Override
    public void close() {
        parent.close();
    }
}
