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
 * Generic interface for providing a remote service.
 *
 * @param <T> Type parameter for service request message
 * @param <S> Type parameter for service response message
 */
public interface IServiceServer<T, S> {

    /**
     * Set the callback which this service will call when requested
     * @param callback Which is triggered on a service request
     */
    void setCallback(OnServiceRequestCallback<T,S> callback);

    /**
     * Get the callback which this service will call when requested
     * @return Callback which is triggered on a service request
     */
    OnServiceRequestCallback<T,S>  getCallback();
}
