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

/**
 * This is the helper class for encoding DDateTime message.
 * All fields' unit in this class match the units in J2735 message.
 */
public class DDateTimeHelper {
    
    protected static final int DYEAR_UNKNOW = 0;
    protected static final int DYEAR_MAX = 0;
    protected static final int DMONTH_UNKNOW = 0;
    protected static final int DMONTH_MAX = 12;
    protected static final int DDAY_UNKNOW = 0;
    protected static final int DDAY_MAX = 31;
    protected static final int DHOUR_UNKNOW = 31;
    protected static final int DHOUR_MIN = 0;
    protected static final int DHOUR_MAX = 23;
    protected static final int DMINUTE_UNKNOW = 60;
    protected static final int DMINUTE_MIN = 0;
    protected static final int DMINUTE_MAX = 59;
    protected static final int DSECOND_MAX = 65535;
    protected static final int DSECOND_MIN = 0;
    protected static final int DOFFSET_UNKNOW = 0;
    protected static final int DOFFSET_MIN = -840;
    protected static final int DOFFSET_MAX = 840;
    
    private int[] timestamp = {DYEAR_UNKNOW, DMONTH_UNKNOW, DDAY_UNKNOW, DHOUR_UNKNOW,
                               DMINUTE_UNKNOW, DSECOND_MAX, DOFFSET_UNKNOW};
    
    public DDateTimeHelper(DateTime dateTime) {
        this.setTimestamp(dateTime);
    }

    public void setTimestamp(DateTime timestamp) {
        if(timestamp.getYear() >= DYEAR_UNKNOW && timestamp.getYear() <= DYEAR_MAX) {
            this.timestamp[0] = timestamp.getYear();
        }
        if(timestamp.getMonth() >= DMONTH_UNKNOW && timestamp.getMonth() <= DMONTH_MAX) {
            this.timestamp[1] = timestamp.getMonth();
        }
        if(timestamp.getDay() >= DDAY_UNKNOW && timestamp.getDay() <= DDAY_MAX) {
            this.timestamp[2] = timestamp.getDay();
        }
        if(timestamp.getHour() >= DHOUR_MIN && timestamp.getHour() <= DHOUR_MAX) {
            this.timestamp[3] = timestamp.getHour();
        }
        if(timestamp.getMinute() >= DMINUTE_MIN && timestamp.getMinute() <= DMINUTE_MAX) {
            this.timestamp[4] = timestamp.getMinute();
        }
        if(timestamp.getSecond() >= DSECOND_MIN && timestamp.getSecond() <= DSECOND_MAX) {
            this.timestamp[5] = timestamp.getSecond();
        }
        if(timestamp.getOffset() >= DOFFSET_MIN && timestamp.getOffset() <= DOFFSET_MAX) {
            this.timestamp[6] = timestamp.getOffset();
        }
    }

    public int[] getTimestamp() {
        return timestamp;
    }
    
}
