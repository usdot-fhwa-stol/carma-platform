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

package gov.dot.fhwa.saxton.carma.message.helper;

import cav_msgs.MobilityHeader;

/**
 * This is the helper class for encoding Mobility Header message.
 * All fields' unit in this class match the units in J2735 message.
 */
public class MobilityHeaderHelper {

    protected static final String GUID_DEFAULT = "00000000-0000-0000-0000-000000000000";
    protected static final long TIMESTAMP_DEFAULT = 0;
    protected static final int TIMESTAMP_LENGTH = Long.toString(Long.MAX_VALUE).length(); 
    protected static final int GUID_LENGTH = GUID_DEFAULT.length();
    
    protected byte[] senderId = new byte[GUID_LENGTH];
    protected byte[] targetId = new byte[GUID_LENGTH];
    protected byte[] planId = new byte[GUID_LENGTH];
    protected byte[] timestamp = new byte[TIMESTAMP_LENGTH];
    
    public MobilityHeaderHelper() {
        this.setId(GUID_DEFAULT, this.senderId);
        this.setId(GUID_DEFAULT, this.targetId);
        this.setId(GUID_DEFAULT, this.planId);
        this.setTimestamp(TIMESTAMP_DEFAULT);
    }
    
    public MobilityHeaderHelper(MobilityHeader header) {
        this.setId(header.getSenderId(), this.senderId);
        this.setId(header.getRecipientId(), this.targetId);
        this.setId(header.getPlanId(), this.planId);
        this.setTimestamp(header.getTimestamp());
    }
    
    public byte[] getSenderId() {
        return senderId;
    }

    public void setId(String inputId, byte[] field) {
        if(inputId.length() == GUID_LENGTH) {
            char[] tmp = inputId.toCharArray();
            for(int i = 0; i < tmp.length; i++) {
                field[i] = (byte) tmp[i];
            }
        }
    }

    public byte[] getTargetId() {
        return targetId;
    }

    public byte[] getPlanId() {
        return planId;
    }

    public byte[] getTimestamp() {
        return timestamp;
    }

    public void setTimestamp(long timestamp) {
        String number = Long.toString(timestamp);
        int numberOfZero = TIMESTAMP_LENGTH - number.length();
        for(int i = 0; i < numberOfZero; i++) {
            number = "0" + number;
        }
        char[] tmp = number.toCharArray();
        for(int i = 0; i < tmp.length; i++) {
            this.timestamp[i] = (byte) tmp[i];
        }
    }
    
}
