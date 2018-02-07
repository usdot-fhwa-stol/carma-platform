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
 * Generic class for sharing underlying resources between multiple {@link IService} instances
 *
 * @param <T> Type parameter for the request type of the service
 * @param <S> Type parameter for the response type of the service
 */
public interface IServiceChannel<T, S> {
    /**
     * Gets a new {@link IService} instance that can utilize the underlying resource
     *
     * @return
     */
    IService<T, S> getService();

    /**
     * Receive notification that a client no longer needs the resources. If there are no more clients,
     * shut down the underlying resources.
     */
    void notifyClientShutdown();

    /**
     * Check whether the underlying resource is still available
     */
    boolean isOpen();

    /**
     * Close the underlying resource
     */
    void close();
}
