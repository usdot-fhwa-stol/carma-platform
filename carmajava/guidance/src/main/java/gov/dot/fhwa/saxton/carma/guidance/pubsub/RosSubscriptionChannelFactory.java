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

import org.ros.node.ConnectedNode;
import gov.dot.fhwa.saxton.carma.guidance.GuidanceExceptionHandler;

/**
 * Concrete ROS implementation of the logic outlined in {@link ISubscriptionChannelFactory}
 *
 * Uses an {@link ConnectedNode} instance to  create {@link org.ros.node.topic.Subscriber} instance
 * for use by {@link RosSubscriptionChannel} instances and their children {@link RosSubscriber}
 */
public class RosSubscriptionChannelFactory implements ISubscriptionChannelFactory {
    protected ConnectedNode node;
    protected GuidanceExceptionHandler exceptionHandler;

    public RosSubscriptionChannelFactory(ConnectedNode node, GuidanceExceptionHandler exceptionHandler) {
        this.node = node;
        this.exceptionHandler = exceptionHandler;
    }

    @Override public <T> ISubscriptionChannel<T> newSubscriptionChannel(String topic, String type) {
        return (ISubscriptionChannel<T>) new RosSubscriptionChannel<>(
            node.newSubscriber(topic, type), exceptionHandler);
    }
}
