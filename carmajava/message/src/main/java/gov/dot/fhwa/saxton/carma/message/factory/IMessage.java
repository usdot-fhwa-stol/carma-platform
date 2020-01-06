/*
 * Copyright (C) 2018-2020 LEIDOS.
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

package gov.dot.fhwa.saxton.carma.message.factory;

import org.ros.internal.message.Message;

import cav_msgs.ByteArray;

public interface IMessage<T> {
    /**
     * This method will take a ROS message and encode it into a binary ByteArray in message container
     * @param plain ROS message the message to be encoded
     * @return null means there is an error
     */
    public MessageContainer encode(Message plainMessage);
    
    /**
     * This method will take a ROS byteArray and decode it to a plain message in message container
     * @param binary ROS message the message to be decoded
     * @return null means there is an error
     */
    public MessageContainer decode(ByteArray binaryMessage);
}
