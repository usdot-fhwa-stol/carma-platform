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

import cav_msgs.DateTime;
import cav_msgs.MobilityHeader;
import cav_msgs.MobilityIntro;

/**
 * This is the helper class for encoding Mobility Introduction message.
 * All fields' unit in this class match the units in J2735 message.
 */
public class MobilityIntroductionMessageHelper {
    
    protected static final int UINT8_MAX = 255;
    protected static final int UNKNOWN_VEHICLE_TYPE = 0;
    protected static final int ROADWAY_ID_SIZE_MIN = 1;
    protected static final int ROADWAY_ID_SIZE_MAX = 50;
    protected static final int POSITION_UNKNOWN = 10001;
    protected static final int POSITION_MIN = 0;
    protected static final int POSITION_MAX = 10000;
    protected static final int LANE_UNKNOWN = 0;
    protected static final int LANE_MAX = 255;
    protected static final int PLAN_PARAM_UNKNOWN = -32768;
    protected static final int PLAN_PARAM_MIN = -32767;
    protected static final int PLAN_PARAM_MAX = 32767;
    protected static final int CAPABILITIES_SIZE_MIN = 1;
    protected static final int CAPABILITIES_SIZE_MAX = 100;
    
    private MobilityHeaderHelper headerHelper = null;
    private int vehicleType = UNKNOWN_VEHICLE_TYPE;
    private int[] roadwayId = {0};
    private int position = POSITION_UNKNOWN;
    private int laneId = LANE_UNKNOWN;
    private int speed = BSMMessageHelper.SPEED_UNAVAILABLE;
    private int planType = 0;
    private int planParam = PLAN_PARAM_UNKNOWN;
    private int[] publicKey = new int[64];
    private DDateTimeHelper expiration = null;
    private int[] capabilities = {0};
    
    public MobilityIntroductionMessageHelper(MobilityIntro intro) {
        this.setHeaderHelper(intro.getHeader());
        this.setVehicleType(intro.getMyEntityType().getType());
        this.setRoadwayId(intro.getMyRoadwayLink());
        this.setPosition(intro.getMyRoadwayLinkPosition());
        this.setLaneId(intro.getMyLaneId());
        this.setSpeed(intro.getForwardSpeed());
        this.setPlanType(intro.getPlanType().getType());
        this.setPlanParam(intro.getProposalParam());
        byte[] tempKey = new byte[64];
        for(int i = 0; i < intro.getMyPublicKey().capacity(); i++) {
            tempKey[i] = intro.getMyPublicKey().getByte(i);
        }
        this.setPublicKey(tempKey);
        this.setExpiration(intro.getExpiration());
        this.setCapabilities(intro.getCapabilities());
    }

    public void setHeaderHelper(MobilityHeader header) {
        this.headerHelper = new MobilityHeaderHelper(header);
    }
    
    public void setVehicleType(byte vehicleType) {
        this.vehicleType = vehicleType;
    }
    
    public void setRoadwayId(String roadwayId) {
        if(roadwayId.length() == 0) {
            return;
        }
        int size = roadwayId.length() <= ROADWAY_ID_SIZE_MAX ? roadwayId.length() : ROADWAY_ID_SIZE_MAX;
        this.roadwayId = new int[size];
        for(int i = 0; i < this.roadwayId.length; i++) {
            this.roadwayId[i] = roadwayId.charAt(i);
        }
    }
    
    public void setPosition(short position) {
        if(position >= POSITION_MIN && position <= POSITION_MAX) {
            this.position = position;
        }       
    }
    
    public void setLaneId(byte laneId) {
        if(laneId >= LANE_UNKNOWN && laneId <= LANE_MAX) {
            this.laneId = laneId;
        }
    }
    
    public void setSpeed(float speed) {
        int integer_speed_input = (int) (speed * 50);
        if(integer_speed_input >= BSMMessageHelper.SPEED_MIN && integer_speed_input <= BSMMessageHelper.SPEED_MAX) {
            this.speed = integer_speed_input;
        }
    }
    
    public void setPlanParam(short planParam) {
        if(planParam >= PLAN_PARAM_MIN && planParam <= PLAN_PARAM_MAX) {
            this.planParam = planParam;
        }
    }
    
    public void setPlanType(byte planType) {
        this.planType = planType;
    }
    
    public void setPublicKey(byte[] publicKey_input) {
        for(int i = 0; i < this.publicKey.length; i++) {
            int keyByte;
            if(publicKey_input[i] < 0) {
                keyByte = 1 + UINT8_MAX + publicKey_input[i]; 
            } else {
                keyByte = publicKey_input[i];
            }
            if(keyByte >= 0 && keyByte <= UINT8_MAX) {
                this.publicKey[i] = keyByte;
            }
        }
    }
    
    public void setExpiration(DateTime expiration) {
        this.expiration = new DDateTimeHelper(expiration);
    }
    
    public void setCapabilities(String capabilities) {
        if(capabilities.length() < CAPABILITIES_SIZE_MIN) {
            return;
        }
        int size = capabilities.length() <= CAPABILITIES_SIZE_MAX ? capabilities.length() : CAPABILITIES_SIZE_MAX;
        this.capabilities = new int[size];
        for(int i = 0; i < this.capabilities.length; i++) {
            this.capabilities[i] = capabilities.charAt(i);
        }
    }
    
    public MobilityHeaderHelper getHeaderHelper() {
        return headerHelper;
    }

    public int getVehicleType() {
        return vehicleType;
    }

    public int[] getRoadwayId() {
        return roadwayId;
    }

    public int getPosition() {
        return position;
    }

    public int getLaneId() {
        return laneId;
    }

    public int getSpeed() {
        return speed;
    }
    
    public int getPlanType() {
        return planType;
    }
    
    public int getPlanParam() {
        return planParam;
    }

    public int[] getPublicKey() {
        return publicKey;
    }

    public DDateTimeHelper getExpiration() {
        return expiration;
    }

    public int[] getCapabilities() {
        return capabilities;
    }
    
}
