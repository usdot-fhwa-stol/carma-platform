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
 * Generic means to create new {@link IServiceServerChannel} instances for new services
 */
public interface IServiceServerChannelFactory {
    /**
     * Get a new {@link IServiceServerChannel} instance for the specified topic and type
     *
     * @param topic The URL for the service topic
     * @param type  The String identifier for the service dialogue type
     * @param <T>   Type parameter for the service request message
     * @param <S>   Type parameter for the service response message
     * 
     * @return Initialized {@link IServiceServerChannel} instance
     */
    <T, S> IServiceServerChannel<T, S> newServiceServerChannel(String topic, String type, OnServiceRequestCallback<T,S> callback);
}
