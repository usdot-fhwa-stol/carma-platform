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

/**
 * Generic interface for resource management related to data publication for a topic
 *
 * @param <T> Type parameter for the message type being published
 */
public interface IPublicationChannel<T> {
    /**
     * Get a new publisher instance capable of writing data messages to the shared resource.
     */
    IPublisher<T> getPublisher();

    /**
     * Check if the underlying resource is still available.
     */
    boolean isOpen();

    /**
     * Close the underlying resource, preventing further operations on this instance.
     */
    void close();

    /**
     * Receive a notification from an {@link IPublisher<T>} associated with this channel that it no
     * longer has need of the resource. If no IPublisher's remain, close the resource.
     */
    void notifyClientShutdown();
}
