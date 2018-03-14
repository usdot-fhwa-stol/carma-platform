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
import org.ros.message.MessageFactory;
import org.ros.node.ConnectedNode;

import cav_msgs.ByteArray;
import gov.dot.fhwa.saxton.carma.rosutils.SaxtonLogger;

/**
 * This class is the actual worker for encoding and decoding Mobility Request message
 * by using J2735 compiler shared library.
 */
public class MobilityRequest implements IMessage<MobilityRequest>{
    
    protected ConnectedNode node_;
    protected SaxtonLogger log_;
    protected MessageFactory messageFactory_;
    
    public MobilityRequest(ConnectedNode node, SaxtonLogger log, MessageFactory messageFactory) {
        this.node_ = node;
        this.log_ = log;
        this.messageFactory_ = messageFactory;
    }

    // Load libasn1c.so external C library
    static {
        try {
            System.loadLibrary("asn1c");
        } catch (Exception e) {
            System.out.println("Exception trapped while trying to load the asn1c library" + e.toString());
            e.printStackTrace();
        }
    }
    
    /**
     * This is the declaration for native method. It will take data from MobilityRequest ROS message
     * and return a byte array including the encoded message. Because of the efficiency of JNI method
     * call, it takes fields directly instead of a single MobilityRequest object.
     * 
     * @return encoded MobilityRequest Message
     */
    private native byte[] encodeMobilityRequest(
            byte[] senderId, byte[] targetId, byte[] senderBSMId, byte[] planId, byte[] timestamp, byte[] strategy,
            int planType, int urgency, int currentX, int currentY, int currentZ, byte[] currentT, byte[] strategyParams,
            int startX, int startY, int startZ, byte[] startT, int[][] offsets, byte[] expiration);
    
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
