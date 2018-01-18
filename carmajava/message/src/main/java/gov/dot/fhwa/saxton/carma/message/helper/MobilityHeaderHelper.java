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

package gov.dot.fhwa.saxton.carma.message.helper;

import java.util.Arrays;

import org.jboss.netty.buffer.ChannelBuffer;

import cav_msgs.DateTime;
import cav_msgs.MobilityHeader;

/**
 * This is the helper class for encoding Mobility Header message.
 * All fields' unit in this class match the units in J2735 message.
 */
public class MobilityHeaderHelper {

    protected static final int GUI_BYTE_MIN = 0;
    protected static final int GUI_BYTE_MAX = 255;
    
    private int[] senderId = new int[16];
    private int[] targetId = new int[16];
    private int[] planId = new int[16];
    private DDateTimeHelper timestamp = null;
    
    public MobilityHeaderHelper(MobilityHeader header) {
        Arrays.fill(senderId, GUI_BYTE_MIN);
        Arrays.fill(targetId, GUI_BYTE_MIN);
        Arrays.fill(planId, GUI_BYTE_MIN);
        ChannelBuffer senderIdBuffer = header.getSenderId();
        byte[] sendIdInput = new byte[senderIdBuffer.capacity()];
        for(int i = 0; i < senderIdBuffer.capacity(); i++) {
            sendIdInput[i] = senderIdBuffer.getByte(i);
        }
        this.setId(sendIdInput, this.senderId);
        ChannelBuffer targetIdBuffer = header.getRecipientId();
        byte[] targetIdInput = new byte[targetIdBuffer.capacity()];
        for(int i = 0; i < targetIdBuffer.capacity(); i++) {
            targetIdInput[i] = targetIdBuffer.getByte(i);
        }
        this.setId(targetIdInput, targetId);
        ChannelBuffer planIdBuffer = header.getPlanId();
        byte[] planIdInput = new byte[planIdBuffer.capacity()];
        for(int i = 0; i < planIdBuffer.capacity(); i++) {
            planIdInput[i] = planIdBuffer.getByte(i);
        }
        this.setId(planIdInput, planId);
        this.timestamp = new DDateTimeHelper(header.getTimestamp());
    }

    public void setId(byte[] inputId, int[] fieldId) {
        for(int i = 0; i < fieldId.length; i++) {
            int temp;
            if(inputId[i] < 0) {
                temp = 1 + GUI_BYTE_MAX + inputId[i];
            } else {
                temp = inputId[i];
            }
            if(temp >= GUI_BYTE_MIN && temp <= GUI_BYTE_MAX) {
                fieldId[i] = temp;
            }
        }
    }
    
    public void setTimestamp(DateTime timestamp) {
        this.timestamp = new DDateTimeHelper(timestamp);
    }
    
    public int[] getSenderId() {
        return senderId;
    }
    
    public int[] getTargetId() {
        return targetId;
    }

    public int[] getPlanId() {
        return planId;
    }

    public DDateTimeHelper getTimestamp() {
        return this.timestamp;
    }
    
}
