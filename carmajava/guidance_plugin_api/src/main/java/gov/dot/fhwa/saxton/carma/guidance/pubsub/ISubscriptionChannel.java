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
 * Generic means by which to share an underyling resource for a topic
 *
 * @param <T>
 */
public interface ISubscriptionChannel<T> {
    /**
     * Get an {@link ISubscriber} instance for the configured topic
     */
    ISubscriber<T> getSubscriber();

    /**
     * Check if the underlying resource is still available
     */
    boolean isOpen();

    /**
     * Close the underlying resources
     */
    void close();

    /**
     * Receive notification from a client that it no longer needs the underlying resource. If no
     * more clients are active, close the resource.
     */
    void notifyClientShutdown();
}
