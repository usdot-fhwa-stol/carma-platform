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
 * Generic service result callback
 *
 * @param <T> The expected result type of the service
 */
public interface OnServiceResponseCallback<T> {
    /**
     * Invoked on successful call of the service.
     *
     * @param msg The service's response
     */
    void onSuccess(T msg);

    /**
     * Invoked on failed call of the service
     *
     * @param e The exception that resulted from attempting the service call
     */
    void onFailure(Exception e);
}
