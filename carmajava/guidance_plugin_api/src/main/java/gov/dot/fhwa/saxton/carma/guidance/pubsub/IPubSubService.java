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
 * Generic interface to serve as a seam point for dependency injection. ROS agnostic implementation
 * of the PubSubManager interface such that it can be distributed separately of the rest of Guidance
 */
public interface IPubSubService {
    /**
     * Get access to a new {@link IService} instance for the specified topic and type
     *
     * @param topicUrl The URL of the service topic to access
     * @param type     The String identifier of the service dialog type
     * @param <T>      Type parameter for the request type of the service dialog
     * @param <S>      Type parameter for the response type of the service dialog
     * @return A new IService instance capable of calling the service
     * @throws TopicNotFoundException
     */
    @SuppressWarnings("unchecked") <T, S> IService<T, S> getServiceForTopic(String topicUrl,
        String type) throws TopicNotFoundException;

    /**
     * Get access to a new {@link IServiceServer} instance for the specified service and type
     *
     * @param topicUrl The URL of the service topic to access
     * @param type     The String identifier of the service dialog type
     * @param <T>      Type parameter for the request type of the service dialog
     * @param <S>      Type parameter for the response type of the service dialog
     * @return A new {@link IServiceServer} instance capable of calling the service
     */
    @SuppressWarnings("unchecked") <T, S> IServiceServer<T, S> getServiceServerForTopic(String topicUrl,
        String type, OnServiceRequestCallback<T,S> callback);

    /**
     * Get access to a new {@link ISubscriber} instance for the specified topic and type
     *
     * @param topicUrl The URL of the topic to subscribe to
     * @param type     The String identifier of the topic message type
     * @param <T>      Type parameter for the topic message type
     * @return A new ISubscriber instance capable of listening to the topic
     */
    @SuppressWarnings("unchecked") <T> ISubscriber<T> getSubscriberForTopic(String topicUrl,
        String type);

    /**
     * Get access to a new {@link IPublisher} instance for the specified topic and type
     *
     * @param topicUrl The URL of the topic to publish to
     * @param type     The String identifier of the topic message type
     * @param <T>      Type parameter for the topic message type
     * @return A new IPublisher instance capable of publishing data to the topic
     */
    @SuppressWarnings("unchecked") <T> IPublisher<T> getPublisherForTopic(String topicUrl,
        String type);
}
