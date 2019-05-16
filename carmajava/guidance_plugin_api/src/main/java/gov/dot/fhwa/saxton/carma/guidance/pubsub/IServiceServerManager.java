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

/**
 * Generic means to create new {@link IServiceServerChannel} instances for new services
 */
public interface IServiceServerManager {
    /**
     * Create an {@link IServiceServer} instance for the specified service and type
     *
     * @param topicUrl The URL of the service topic to access
     * @param type     The String identifier of the service dialog type
     * @param <T>      Type parameter for the request type of the service dialog
     * @param <S>      Type parameter for the response type of the service dialog
     */
    @SuppressWarnings("unchecked") <T, S> void createServiceServerForTopic(String topicUrl,
        String type, OnServiceRequestCallback<T,S> callback);
}
