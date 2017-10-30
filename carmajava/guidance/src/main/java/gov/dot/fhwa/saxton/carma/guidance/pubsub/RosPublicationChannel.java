/*
 * Copyright (C) 2017 LEIDOS.
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
 * Manages the underlying resources associated with a given ROS topic publication
 *
 * @param <T>
 */
public class RosPublicationChannel<T> implements IPublicationChannel<T> {
    protected int numOpenChannels = 0;
    protected boolean open = true;
    protected Publisher<T> publisher;

    RosPublicationChannel(Publisher<T> publisher) {
        this.publisher = publisher;
    }

    /**
     * Get a new IPublisher instance for the topic
     */
    @Override public IPublisher<T> getPublisher() {
        numOpenChannels++;
        return new RosPublisher<>(publisher, this);
    }

    /**
     * Notice that an instance has been closed. If there are no more instances shut down the resource
     */
    @Override public void notifyClientShutdown() {
        numOpenChannels--;

        if (numOpenChannels <= 0) {
            close();
        }
    }

    @Override public void close() {

    }

    /**
     * Return whether or not the underlying resource for this RosPublicationChannel has been shut down
     */
    public boolean isOpen() {
        return open;
    }

    /**
     * Get the number of extant IPublisher instances that haven't been closed
     */
    public int getNumOpenChannels() {
        return numOpenChannels;
    }
}
