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

package gov.dot.fhwa.saxton.carma.guidance.pubsub;

/**
 * Generic means to create a new {@link ISubscriptionChannel} for a new topic and type
 */
public interface ISubscriptionChannelFactory {
    /**
     * Create a new {@link ISubscriptionChannel} for the specified topic and type
     *
     * @param topic The URL for the topic to subscribe to
     * @param type  The String identifier for the topic message type
     * @param <T>   Type parameter for the topic message type
     * @return A new ISubscriptionChannel instance configured for the topic
     */
    <T> ISubscriptionChannel<T> newSubscriptionChannel(String topic, String type);
}
