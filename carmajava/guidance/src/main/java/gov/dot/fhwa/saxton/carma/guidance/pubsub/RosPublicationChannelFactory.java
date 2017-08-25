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

import org.ros.node.ConnectedNode;

/**
 * Concrete ROS implementation of the logic from {@link IPublicationChannelFactory}
 *
 * Uses a ROS ConnectedNode instance to generate {@link org.ros.node.topic.Publisher} instances to
 * be shared by {@link RosPublicationChannel} and {@link RosPublisher}
 */
public class RosPublicationChannelFactory implements IPublicationChannelFactory {
    protected ConnectedNode node;

    public RosPublicationChannelFactory(ConnectedNode node) {
        this.node = node;
    }

    @Override public <T> IPublicationChannel<T> newPublicationChannel(String topic, String type) {
        return (IPublicationChannel<T>) new RosPublicationChannel<>(node.newPublisher(topic, type));
    }
}
