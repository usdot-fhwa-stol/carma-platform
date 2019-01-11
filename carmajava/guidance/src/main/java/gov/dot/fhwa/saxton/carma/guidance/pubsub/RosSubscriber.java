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

import java.util.LinkedList;
import java.util.List;

import org.ros.message.MessageListener;
import org.ros.node.topic.Subscriber;
import gov.dot.fhwa.saxton.carma.guidance.*;

/**
 * Concrete ROS implementation of the logic outlined in {@link ISubscriber}
 *
 * Uses a {@link Subscriber} to receive ROS messages from the configured topic
 * @param <T> Type parameter for the message type of the topic
 */
public class RosSubscriber<T> implements ISubscriber<T> {
    protected Subscriber<T> subscriber;
    protected RosSubscriptionChannel<T> parent;
    protected T lastMessage = null;
    protected GuidanceExceptionHandler exceptionHandler;
    protected List<MessageListener<T>> listeners = new LinkedList<>();

    RosSubscriber(Subscriber<T> subscriber, RosSubscriptionChannel<T> parent, GuidanceExceptionHandler exceptionHandler) {
        this.subscriber = subscriber;
        this.parent = parent;
        this.exceptionHandler = exceptionHandler;

        subscriber.addMessageListener(new MessageListener<T>() {
            @Override public void onNewMessage(T t) {
                lastMessage = t;
            }
        });
    }

    @Override public T getLastMessage() {
        return lastMessage;
    }

    @Override public void registerOnMessageCallback(final OnMessageCallback<T> callback) {
    	MessageListener<T> listener = new MessageListener<T>() {
            @Override public void onNewMessage(T t) {
                try {
                    callback.onMessage(t);
                } catch (Exception e) {
                    // Uncaught exception received, throw it up to the top level handler
                    exceptionHandler.handleException(e);
                }
            }
        };
        listeners.add(listener);
        subscriber.addMessageListener(listener);
    }

    @Override public void close() {
    	for(MessageListener<T> listener : listeners) {
    		subscriber.removeMessageListener(listener);
    	}
        parent.notifyClientShutdown();
    }
}
