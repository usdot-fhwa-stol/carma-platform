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

import java.util.concurrent.BlockingQueue;

/**
 * Guidance package PubSubManager component
 *
 * Responsible for allowing communications between Guidance package sub-components
 * and the external ROS network. Presently only a stub to show class communication.
 */
public class PubSubManager {
    public PubSubManager(BlockingQueue<String> messageQueue) {
        this.messageQueue = messageQueue;
    }

    public void publish(String msg) {
        try {
            messageQueue.put(msg);
        } catch (InterruptedException e) {
            // Ignore
        }
    }

    public <T> SubscriptionChannelManager<T> getSubscriptionChannelForTopic(String topicUrl) {
        return new SubscriptionChannelManager<>();
    }

    public <T> PublicationChannelManager<T> getPublicationChannelForTopic(String topicUrl) {
        return new PublicationChannelManager<>();
    }

    // Member Variables
    protected BlockingQueue<String> messageQueue;
}
