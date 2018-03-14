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
    protected static final String STRING_DEFAULT = "[]";
    protected static final String BSM_ID_DEFAULT = "00000000";
    protected static final long TIMESTAMP_DEFAULT = 0;
    protected static final int TIMESTAMP_LENGTH = Long.toString(Long.MAX_VALUE).length(); 
    protected static final int GUID_LENGTH = GUID_DEFAULT.length();
    protected static final int BSM_ID_LENGTH = BSM_ID_DEFAULT.length();
    protected static final int STATIC_ID_MAX_LENGTH = 14;
    
    private byte[] senderId, targetId, hostBSMId, planId, timestamp;
    
    public MobilityHeaderHelper(MobilityHeader header) {
        this.setStaticId(header.getSenderId(), this.senderId);
        this.setStaticId(header.getRecipientId(), this.targetId);
        //this.setId(header.getPlanId(), this.planId);
        this.setTimestamp(header.getTimestamp());
    }

    public void setStaticId(String inputId, byte[] field) {
        char[] tmp;
        if(inputId.length() <= STATIC_ID_MAX_LENGTH) {
            inputId = "[" + inputId + "]";
            tmp = inputId.toCharArray();   
        } else {
            tmp = STRING_DEFAULT.toCharArray();
        }
        field = new byte[tmp.length];
        for(int i = 0; i < tmp.length; i++) {
            field[i] = (byte) tmp[i];
        }
    }

    public byte[] getSenderId() {
        return senderId;
    }
    
    public byte[] getTargetId() {
        return targetId;
    }
    
    public void setBSMId(String inputId, byte[] field) {
        char[] tmp;
        if(inputId.length() == BSM_ID_LENGTH) {
            tmp = inputId.toCharArray();
        } else {
            tmp = BSM_ID_DEFAULT.toCharArray();
        }
        field = new byte[BSM_ID_LENGTH];
        for(int i = 0; i < tmp.length; i++) {
            field[i] = (byte) tmp[i];
        }
    }
    
    public byte[] getBSMId() {
        return hostBSMId;
    }

    public void setPlanId(String inputId, byte[] field) {
        char[] tmp;
        if(inputId.length() == GUID_LENGTH) {
            tmp = inputId.toCharArray();
        } else {
            tmp = GUID_DEFAULT.toCharArray();
        }
        field = new byte[GUID_LENGTH];
        for(int i = 0; i < tmp.length; i++) {
            field[i] = (byte) tmp[i];
        }
    }
    
    public byte[] getPlanId() {
        return planId;
    }

    public void setTimestamp(long timestamp) {
        StringBuffer number = new StringBuffer(Long.toString(timestamp));
        int numberOfZero = TIMESTAMP_LENGTH - number.length();
        for(int i = 0; i < numberOfZero; i++) {
            number.insert(0, '0');
        }
        char[] tmp = number.toString().toCharArray();
        this.timestamp = new byte[tmp.length];
        for(int i = 0; i < tmp.length; i++) {
            this.timestamp[i] = (byte) tmp[i];
        }
    }
    
    public byte[] getTimestamp() {
        return timestamp;
    }
}
