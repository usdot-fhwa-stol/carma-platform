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

import cav_msgs.LocationECEF;

/**
 * This is a helper class for converting LocationECEF MESSAGE into a java
 * helper class which only contains int and byte[] data type.
 */
public class MobilityECEFLocationHelper {

    protected static final int ECEF_XY_MIN = -638363700;
    protected static final int ECEF_XY_MAX = 638363700;
    protected static final int ECEF_Z_MIN = -636225200;
    protected static final int ECEF_Z_MAX = 636225200;
    
    private int ecefX, ecefY, ecefZ;
    private byte[] timestamp;
    
    public MobilityECEFLocationHelper(LocationECEF location) {
        this.setEcefX(location.getEcefX());
        this.setEcefY(location.getEcefY());
        this.setEcefZ(location.getEcefZ());
        this.timestamp = StringConverterHelper.setTimestamp(location.getTimestamp());
    }
    
    public void setEcefX(int x) {
        if(x >= ECEF_XY_MIN && x <= ECEF_XY_MAX) {
            ecefX = x;
        } else {
            ecefX = 0;
        }
    }
    
    public void setEcefY(int y) {
        if(y >= ECEF_XY_MIN && y <= ECEF_XY_MAX) {
            ecefY = y;
        } else {
            ecefY = 0;
        }
    }
    
    public void setEcefZ(int z) {
        if(z >= ECEF_Z_MIN && z <= ECEF_Z_MAX) {
            ecefZ = z;
        } else {
            ecefZ = 0;
        }
    }

    public int getEcefX() {
        return ecefX;
    }

    public int getEcefY() {
        return ecefY;
    }

    public int getEcefZ() {
        return ecefZ;
    }

    public byte[] getTimestamp() {
        return timestamp;
    }

}
