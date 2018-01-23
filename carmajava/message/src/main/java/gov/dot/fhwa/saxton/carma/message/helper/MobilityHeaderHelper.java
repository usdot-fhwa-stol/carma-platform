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

import cav_msgs.MobilityHeader;

/**
 * This is the helper class for encoding Mobility Header message.
 * All fields' unit in this class match the units in J2735 message.
 */
public class MobilityHeaderHelper {

    protected static final char[] GUID_DEFAULT = "00000000-0000-0000-0000-000000000000".toCharArray();
    protected static final char[] TIMESTAMP_DEFAULT = "0000000000000000000".toCharArray();
    protected static final int TIMESTAMP_LENGTH = TIMESTAMP_DEFAULT.length; 
    protected static final int GUID_LENGTH = GUID_DEFAULT.length;
    
    protected char[] senderId = GUID_DEFAULT;
    protected char[] targetId = GUID_DEFAULT;
    protected char[] planId = GUID_DEFAULT;
    protected char[] timestamp = TIMESTAMP_DEFAULT;
    
    public MobilityHeaderHelper() {
        
    }
    
    public MobilityHeaderHelper(MobilityHeader header) {
        this.setSenderId(header.getSenderId());
        this.setTargetId(header.getRecipientId());
        this.setPlanId(header.getPlanId());
        this.setTimestamp(header.getTimestamp());
    }
    
    public char[] getSenderId() {
        return senderId;
    }

    public void setSenderId(String senderId) {
        if(senderId.length() == GUID_LENGTH) {
            this.senderId = senderId.toCharArray();
        }
    }

    public char[] getTargetId() {
        return targetId;
    }

    public void setTargetId(String targetId) {
        if(targetId.length() == GUID_LENGTH) {
            this.targetId = targetId.toCharArray();
        }
    }

    public char[] getPlanId() {
        return planId;
    }

    public void setPlanId(String planId) {
        if(planId.length() == GUID_LENGTH) {
            this.planId = planId.toCharArray();
        }
    }

    public char[] getTimestamp() {
        return timestamp;
    }

    public void setTimestamp(long timestamp) {
        String number = Long.toString(timestamp);
        int numberOfZero = number.length() - TIMESTAMP_LENGTH;
        for(int i = 0; i < numberOfZero; i++) {
            number = "0" + number;
        }
        this.timestamp = number.toCharArray();
    }
    
}
