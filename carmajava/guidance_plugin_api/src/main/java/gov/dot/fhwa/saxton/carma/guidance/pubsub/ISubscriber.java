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

/**
 * Generic interface for subscribing to a data stream. Designed to share an underlying resource to
 * avoid duplication.
 *
 * @param <T> Type parameter for the message being subscribed to
 */
public interface ISubscriber<T> {
    /**
     * Get the last message received on this channel if one has been received.
     *
     * @return Either the last received message or null if none has been received yet
     */
    T getLastMessage();

    /**
     * Add a callback to be executed whenever a new message is received for this ISubscriptionChannel
     *
     * @param callback The callback to be executed
     */
    void registerOnMessageCallback(OnMessageCallback<T> callback);

    /**
     * Notify this ISubscriptionChannel instance's parent of this instance's closure. This will not necessarily close
     * the underlying resources associated with this IService.
     */
    void close();
}
