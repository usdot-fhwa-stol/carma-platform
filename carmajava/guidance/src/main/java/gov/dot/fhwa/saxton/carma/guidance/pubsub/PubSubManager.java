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

import java.util.HashMap;
import java.util.Map;

import gov.dot.fhwa.saxton.carma.guidance.GuidanceExceptionHandler;

/**
 * Guidance package PubSubManager component
 * <p>
 * Responsible for allowing communications between Guidance package sub-components
 * and the external ROS network. Presently only a stub to show class communication.
 *
 * @SupressWarnings is used because there's no clean way to ensure type-correctness inside Java's type system. Since the
 * topic names are strings and the topic type names are strings and the mapping of topic -> type is defined externally
 * to Java's type system we can't really leverage Java types to ensure correctness. Vigilance will be required to only
 * request types from topics that supply them per our own documentation.
 */
public class PubSubManager implements IPubSubService {
    // Member Variables
    protected ISubscriptionChannelFactory subFactory;
    protected IPublicationChannelFactory pubFactory;
    protected IServiceChannelFactory srvFactory;
    protected Map<String, IPublicationChannel> pubChannelManagers;
    protected Map<String, ISubscriptionChannel> subChannelManagers;
    protected Map<String, IServiceChannel> srvManagers;

    public PubSubManager(ISubscriptionChannelFactory subFactory, IPublicationChannelFactory pubFactory,
            IServiceChannelFactory srvFactory) {

        this.subFactory = subFactory;
        this.pubFactory = pubFactory;
        this.srvFactory = srvFactory;

        pubChannelManagers = new HashMap<>();
        subChannelManagers = new HashMap<>();
        srvManagers = new HashMap<>();
    }

    /**
     * Get access to an IService instance
     *
     * @param topicUrl A URL identifying the ROS topic for the desired service
     * @param type     The string identifier of the message type
     * @param <T>      Type parameter for the request message
     * @param <S>      Type parameter for the response message
     * @return An IService instance that can call the service
     */
    @Override
    @SuppressWarnings("unchecked")
    public <T, S> IService<T, S> getServiceForTopic(String topicUrl, String type) throws TopicNotFoundException {
        if (srvManagers.containsKey(topicUrl) && srvManagers.get(topicUrl).isOpen()) {
            return srvManagers.get(topicUrl).getService();
        } else {
            IServiceChannel<T, S> mgr = srvFactory.newServiceChannel(topicUrl, type);
            srvManagers.put(topicUrl, mgr);
            return mgr.getService();
        }
    }

    /**
     * Get access to an ISubscriber instance
     *
     * @param topicUrl A URL identifying the ROS topic for the subscription
     * @param type     The string identifier of the message type
     * @param <T>      Type parameter of the topic message
     * @return An ISubscriber instance that has subscription access to the topic
     */
    @Override
    @SuppressWarnings("unchecked")
    public <T> ISubscriber<T> getSubscriberForTopic(String topicUrl, String type) {
        if (subChannelManagers.containsKey(topicUrl) && subChannelManagers.get(topicUrl).isOpen()) {
            return subChannelManagers.get(topicUrl).getSubscriber();
        } else {
            ISubscriptionChannel<T> mgr = subFactory.newSubscriptionChannel(topicUrl, type);
            subChannelManagers.put(topicUrl, mgr);
            return mgr.getSubscriber();
        }
    }

    /**
     * Get access to an IPublisher instance
     *
     * @param topicUrl A URL identifying the ROS topic for the publication
     * @param type     The string identifier of the message type
     * @param <T>      Type parameter of the topic message
     * @return An IPublisher instance that has publish access to the topic
     */
    @Override
    @SuppressWarnings("unchecked")
    public <T> IPublisher<T> getPublisherForTopic(String topicUrl, String type) {
        if (pubChannelManagers.containsKey(topicUrl) && pubChannelManagers.get(topicUrl).isOpen()) {
            return pubChannelManagers.get(topicUrl).getPublisher();
        } else {
            IPublicationChannel<T> mgr = pubFactory.newPublicationChannel(topicUrl, type);
            pubChannelManagers.put(topicUrl, mgr);
            return mgr.getPublisher();
        }
    }
}
