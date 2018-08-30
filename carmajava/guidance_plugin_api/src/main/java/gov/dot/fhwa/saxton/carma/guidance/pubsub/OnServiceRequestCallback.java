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
 * Generic service request callback
 *
 * @param <T> The expected request type of the service
 * @param <S> The expected response type of the service
 */
public interface OnServiceRequestCallback<T,S> {
    /**
     * Called when a new request is made to this service
     * Returns the result of this request
     * 
     * @param request The request of type T
     * @param The response of type S which will be sent. This should be modified to the desired result.
     */
    void onRequest(T request, S response);
}
