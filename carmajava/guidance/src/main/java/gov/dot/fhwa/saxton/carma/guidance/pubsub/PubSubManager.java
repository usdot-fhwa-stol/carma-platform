/*
 * TODO: Copyright (C) 2017 LEIDOS.
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

//TODO: Naming convention of "package gov.dot.fhwa.saxton.carmajava.<template>;"
//Originally "com.github.rosjava.carmajava.template;"
package gov.dot.fhwa.saxton.carma.guidance.pubsub;

import org.ros.exception.ServiceNotFoundException;
import org.ros.node.ConnectedNode;

import java.util.HashMap;
import java.util.Map;

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
public class PubSubManager {
    public PubSubManager(ConnectedNode node) {
        this.node = node;
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
    @SuppressWarnings("unchecked") public <T, S> IService<T, S> getService(String topicUrl,
        String type) throws ServiceNotFoundException {
        if (srvManagers.containsKey(topicUrl) && srvManagers.get(topicUrl).isOpen()) {
            return srvManagers.get(topicUrl).getNewChannel();
        } else {
            ServiceManager<T, S> mgr = new ServiceManager<>(node, topicUrl, type);
            mgr.openServiceClient();
            srvManagers.put(topicUrl, mgr);
            return mgr.getNewChannel();
        }
    }

    /**
     * Get access to an ISubscriptionChannel instance
     *
     * @param topicUrl A URL identifying the ROS topic for the subscription
     * @param type     The string identifier of the message type
     * @param <T>      Type parameter of the topic message
     * @return An ISubscriptionChannel instance that has subscription access to the topic
     */
    @SuppressWarnings("unchecked")
    public <T> ISubscriptionChannel<T> getSubscriptionChannelForTopic(String topicUrl,
        String type) {
        if (subChannelManagers.containsKey(topicUrl) && subChannelManagers.get(topicUrl).isOpen()) {
            return subChannelManagers.get(topicUrl).getNewChannel();
        } else {
            SubscriptionChannelManager<T> mgr =
                new SubscriptionChannelManager<>(node, topicUrl, type);
            subChannelManagers.put(topicUrl, mgr);
            return mgr.getNewChannel();
        }
    }

    /**
     * Get access to an IPublicationChannel instance
     *
     * @param topicUrl A URL identifying the ROS topic for the publication
     * @param type     The string identifier of the message type
     * @param <T>      Type parameter of the topic message
     * @return An IPublicationChannel instance that has publish access to the topic
     */
    @SuppressWarnings("unchecked") public <T> IPublicationChannel<T> getPublicationChannelForTopic(
        String topicUrl, String type) {
        if (pubChannelManagers.containsKey(topicUrl) && pubChannelManagers.get(topicUrl).isOpen()) {
            return pubChannelManagers.get(topicUrl).getNewChannel();
        } else {
            PublicationChannelManager<T> mgr =
                new PublicationChannelManager<>(node, topicUrl, type);
            pubChannelManagers.put(topicUrl, mgr);
            return mgr.getNewChannel();
        }
    }

    // Member Variables
    protected ConnectedNode node;
    protected Map<String, PublicationChannelManager> pubChannelManagers;
    protected Map<String, SubscriptionChannelManager> subChannelManagers;
    protected Map<String, ServiceManager> srvManagers;
}
