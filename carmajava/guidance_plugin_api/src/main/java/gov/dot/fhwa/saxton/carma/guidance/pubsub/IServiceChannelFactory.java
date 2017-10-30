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
 * Generic means to create new {@link IServiceChannel} instances for new topics/services
 */
public interface IServiceChannelFactory {
    /**
     * Get a new {@link IServiceChannel} instance for the specified topic and type
     *
     * @param topic The URL for the service topic
     * @param type  The String identifier for the service dialogue type
     * @param <T>   Type parameter for the service request message
     * @param <S>   Type parameter for the service response message
     * @throws TopicNotFoundException If there is not a valid service residing at the specified URL
     */
    <T, S> IServiceChannel<T, S> newServiceChannel(String topic, String type)
        throws TopicNotFoundException;
}
