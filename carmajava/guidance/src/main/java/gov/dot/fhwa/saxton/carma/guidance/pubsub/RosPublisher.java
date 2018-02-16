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

import org.ros.node.topic.Publisher;

/**
 * Concrete ROS implementation of the functionality outlined in {@link IPublisher}
 *
 * Shares a {@link Publisher} amongst it's coworkers (other RosPublisher instances for same topic)
 * and it's parent {@link RosPublicationChannel}
 *
 * @param <T> Type parameter for the published message type
 */
public class RosPublisher<T> implements IPublisher<T> {

    protected Publisher<T> publisher;
    protected IPublicationChannel<T> parent;

    public RosPublisher(Publisher<T> publisher, IPublicationChannel<T> parent) {
        this.publisher = publisher;
        this.parent = parent;
    }

    @Override public T newMessage() {
        return publisher.newMessage();
    }

    @Override public void publish(T msg) {
        publisher.publish(msg);
    }

    @Override public void close() {
        parent.notifyClientShutdown();
    }
}
