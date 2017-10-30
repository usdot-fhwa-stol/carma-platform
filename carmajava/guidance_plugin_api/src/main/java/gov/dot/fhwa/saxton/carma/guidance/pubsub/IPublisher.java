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
 * Generic interface for publishing data to some kind of data stream. Designed to share a single
 * underyling connection resource to avoid duplication.
 *
 * @param <T> Type parameter for the message being published
 */
public interface IPublisher<T> {
    /**
     * Generate a new (empty) message instance to have it's fields populated before transmission.
     *
     * @return An empty message of type T
     */
    T newMessage();

    /**
     * Publish a message on this channel
     *
     * @param msg The message to publish
     */
    void publish(T msg);

    /**
     * Notify this IPublicationChannel instance's parent of this instance's closure. This will not necessarily close the
     * underlying resources associated with this IService.
     */
    void close();
}
