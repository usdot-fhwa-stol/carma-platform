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

import org.ros.node.ConnectedNode;

import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.BlockingQueue;

/**
 * Guidance package PubSubManager component
 *
 * Responsible for allowing communications between Guidance package sub-components
 * and the external ROS network. Presently only a stub to show class communication.
 */
public class PubSubManager {
    public PubSubManager(ConnectedNode node) {
        this(new ArrayBlockingQueue<String>(64));
        this.node = node;
    }

    public PubSubManager(BlockingQueue<String> messageQueue) {
        this.messageQueue = messageQueue;
        pubChannelManagers = new HashMap<>();
        subChannelManagers = new HashMap<>();
        srvManagers = new HashMap<>();
    }

    @Deprecated
    public void publish(String msg) {
        try {
            messageQueue.put(msg);
        } catch (InterruptedException e) {
            // Ignore
        }
    }

    @SuppressWarnings("unchecked")
    public <T, S> IService<T, S> getService(String topicUrl, String type) {
        if (srvManagers.containsKey(topicUrl)) {
            return srvManagers.get(topicUrl).getNewChannel();
        } else {
            ServiceManager<T, S> mgr = new ServiceManager<>(node, topicUrl, type);
            srvManagers.put(topicUrl, mgr);
            return mgr.getNewChannel();
        }
    }

    @SuppressWarnings("unchecked")
    public <T> ISubscriptionChannel<T> getSubscriptionChannelForTopic(String topicUrl, String type) {
        if (subChannelManagers.containsKey(topicUrl)) {
            return  subChannelManagers.get(topicUrl).getNewChannel();
        } else {
            SubscriptionChannelManager<T> mgr = new SubscriptionChannelManager<>(node, topicUrl, type);
            subChannelManagers.put(topicUrl, mgr);
            return mgr.getNewChannel();
        }
    }

    @SuppressWarnings("unchecked")
    public <T> IPublicationChannel<T> getPublicationChannelForTopic(String topicUrl, String type) {
        if (pubChannelManagers.containsKey(topicUrl)) {
            return  pubChannelManagers.get(topicUrl).getNewChannel();
        } else {
            PublicationChannelManager<T> mgr = new PublicationChannelManager<>(node, topicUrl, type);
            pubChannelManagers.put(topicUrl, mgr);
            return mgr.getNewChannel();
        }
    }

    // Member Variables
    protected BlockingQueue<String> messageQueue;
    protected ConnectedNode node;
    protected Map<String, PublicationChannelManager> pubChannelManagers;
    protected Map<String, SubscriptionChannelManager> subChannelManagers;
    protected Map<String, ServiceManager> srvManagers;
}
