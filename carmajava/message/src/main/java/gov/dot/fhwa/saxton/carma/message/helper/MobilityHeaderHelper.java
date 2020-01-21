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

package gov.dot.fhwa.saxton.carma.message.helper;

import cav_msgs.MobilityHeader;

/**
 * This is the helper class for encoding Mobility Header message.
 * All fields' unit in this class match the units in J2735 message.
 */
public class MobilityHeaderHelper {
    
    protected static final int STATIC_ID_MAX_LENGTH = 16;
    protected static final String BSM_ID_DEFAULT = "00000000";
    protected static final int BSM_ID_LENGTH = BSM_ID_DEFAULT.length();
    protected static final String GUID_DEFAULT = "00000000-0000-0000-0000-000000000000";
    protected static final int GUID_LENGTH = GUID_DEFAULT.length();
    
    private byte[] senderId, targetId, hostBSMId, planId, timestamp;
    
    public MobilityHeaderHelper(MobilityHeader header) {
        this.senderId = StringConverterHelper.setDynamicLengthString(header.getSenderId(), STATIC_ID_MAX_LENGTH);
        this.targetId = StringConverterHelper.setDynamicLengthString(header.getRecipientId(), STATIC_ID_MAX_LENGTH);
        this.hostBSMId = StringConverterHelper.setFixedLengthString(header.getSenderBsmId(), BSM_ID_LENGTH, BSM_ID_DEFAULT);
        this.planId = StringConverterHelper.setFixedLengthString(header.getPlanId(), GUID_LENGTH, GUID_DEFAULT);
        this.timestamp = StringConverterHelper.setTimestamp(header.getTimestamp());
    }

    public byte[] getSenderId() {
        return senderId;
    }
    
    public byte[] getTargetId() {
        return targetId;
    }
    
    public byte[] getBSMId() {
        return hostBSMId;
    }
    
    public byte[] getPlanId() {
        return planId;
    }
    
    public byte[] getTimestamp() {
        return timestamp;
    }
}
