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

package gov.dot.fhwa.saxton.carma.message.factory;

import org.ros.internal.message.Message;

import cav_msgs.ByteArray;
import cav_msgs.MobilityOperation;

/**
 * This class is the actual worker for encoding and decoding Mobility
 * Operation message by using J2735 compiler shared library.
 */
public class MobilityOperationMessage implements IMessage<MobilityOperation> {

    @Override
    public MessageContainer encode(Message plainMessage) {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public MessageContainer decode(ByteArray binaryMessage) {
        // TODO Auto-generated method stub
        return null;
    }

}
