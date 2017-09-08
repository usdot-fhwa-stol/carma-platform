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

package gov.dot.fhwa.saxton.carma.guidance.pubsub;

import org.ros.exception.ServiceNotFoundException;
import org.ros.node.ConnectedNode;
import org.ros.node.service.ServiceClient;

/**
 * Concrete ROS implementation of the logic outlined in {@link IServiceChannelFactory}
 *
 * Uses an {@link ConnectedNode} instance to create {@link ServiceClient} instances for use by
 * {@link RosServiceChannel} instances and their children {@link RosService} instances
 */
public class RosServiceChannelFactory implements IServiceChannelFactory {
    protected ConnectedNode node;
    protected static final long SERVICE_LOCATE_TIMEOUT = 5000;

    public RosServiceChannelFactory(ConnectedNode node) {
        this.node = node;
    }

    @Override public <T, S> IServiceChannel<T, S> newServiceChannel(String topic, String type)
        throws TopicNotFoundException {
        /*
         * Duplication of SaxtonBaseNode's waitForService functionality
         *
         * I don't want the GuidancePlugins to have any sort of dependency on the Guidance node
         * itself, so they can't have a reference to the SaxtonBaseNode instance. The logic is
         * simple enough so I've just reimplemented it here.
         */

        boolean serviceFound = false;
        IServiceChannel<T, S> out = null;
        long curTime = System.currentTimeMillis();
        long timeAccumulator = 0;
        while (!serviceFound && timeAccumulator < SERVICE_LOCATE_TIMEOUT) {
            try {
                out = (IServiceChannel<T, S>) new RosServiceChannel<>(node.newServiceClient(topic, type));
                serviceFound = true;
            } catch (ServiceNotFoundException e) {
                serviceFound = false;
            }

            long newTime = System.currentTimeMillis();
            timeAccumulator += newTime - curTime;
            curTime = newTime;
        }

        if (serviceFound) {
            return out;
        } else {
            throw new TopicNotFoundException();
        }
    }
}
