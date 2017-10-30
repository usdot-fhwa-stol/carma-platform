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

import org.ros.node.topic.Subscriber;
import gov.dot.fhwa.saxton.carma.guidance.*;

/**
 * Package private class for use in the PubSubManager
 * <p>
 * Responsible for keeping track of the subscription channel resources associated with any number of ISubscriptionChannels
 * for a given topic.
 *
 * @param <T> Type parameter for the message of the topic
 */
public class RosSubscriptionChannel<T> implements ISubscriptionChannel<T> {
    protected int numOpenChannels = 0;
    protected boolean open = true;
    protected Subscriber<T> subscriber;
    protected GuidanceExceptionHandler exceptionHandler;

    RosSubscriptionChannel(Subscriber<T> subscriber, GuidanceExceptionHandler exceptionHandler) {
        this.subscriber = subscriber;
        this.exceptionHandler = exceptionHandler;
    }

    /**
     * Acquire a new ISubscriber instance
     */
    @Override
    public ISubscriber<T> getSubscriber() {
        numOpenChannels++;
        return new RosSubscriber<>(subscriber, this, exceptionHandler);
    }

    /**
     * Register the destruction of a channel interface instance. If none are open then close the resource.
     */
    @Override
    public void notifyClientShutdown() {
        numOpenChannels--;

        if (numOpenChannels <= 0) {
            close();
        }
    }

    /**
     * Get the number of extant channel instances that haven't been closed yet
     */
    public int getNumOpenChannels() {
        return numOpenChannels;
    }

    /**
     * Return whether or not the underlying resource for this RosPublicationChannel has been shut down
     */
    @Override
    public boolean isOpen() {
        return open;
    }

    @Override
    public void close() {
        open = false;
        subscriber.shutdown();
    }
}
