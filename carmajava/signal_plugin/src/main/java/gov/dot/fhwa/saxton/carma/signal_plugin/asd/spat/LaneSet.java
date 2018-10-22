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

package gov.dot.fhwa.saxton.carma.signal_plugin.asd.spat;

/**
 * Defines a SPAT LaneSet object
 *
 * User: ferenced
 * Date: 1/17/15
 * Time: 12:43 PM
 *
 */
public class LaneSet {
    private int lane;
    private int movement;

    private static final String STRAIGHT = "STRAIGHT";
    private static final String LEFT_TURN = "LEFT TURN";
    private static final String RIGHT_TURN = "RIGHT TURN";
    private static final String U_TURN = "U TURN";
    private static final String SEPARATOR = " / ";

    public LaneSet(int lane, int movement)   {
        this.lane = lane;
        this.movement = movement;
    }

    public int getLane()   { return lane; }
    public int getMovement() { return movement; }

    // Bits (LSB)
    // 0 - Straight
    // 1 = Left Turn
    // 2 = Right Turn
    // 3 = U Turn
    public String getMovementAsString()   {

        StringBuffer sb = new StringBuffer();

        byte byteMovement = (byte) movement;

        if ( (byteMovement & (byte) 0x01) == 0x01)   {
           sb.append(STRAIGHT);
           sb.append(SEPARATOR);
        }
        if ( (byteMovement & (byte) 0x02) == 0x02)   {
            sb.append(LEFT_TURN);
            sb.append(SEPARATOR);
        }
        if ( (byteMovement & (byte) 0x04) == 0x04)   {
            sb.append(RIGHT_TURN);
            sb.append(SEPARATOR);
        }
        if ( (byteMovement & (byte) 0x08) == 0x08)   {
            sb.append(U_TURN);
            sb.append(SEPARATOR);
        }

        return sb.toString();

    }

    /**
     * Identifies whether the lane movement is straight
     *
     * This is the only one of interest for glidepath
     *
     * @return boolean
     */
    public boolean isStraight()    {
        byte byteMovement = (byte) movement;

        return ( (byteMovement & (byte) 0x01) == 0x01 );
    }

}
