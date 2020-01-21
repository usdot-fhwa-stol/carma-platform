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

import java.util.LinkedList;
import java.util.List;

import org.ros.message.MessageFactory;

import cav_msgs.LocationECEF;
import cav_msgs.LocationOffsetECEF;
import cav_msgs.Trajectory;

/**
 * This is a helper class to convert Trajectory ROS message into a java class
 * which only contains int and int[] as data type. This class will be used by
 * other Mobility helper classes.
 */
public class MobilityTrajectoryHelper {
    
    protected static final int XYZ_MIN = -500;
    protected static final int XYZ_MAX = 500;
    protected static final int XYZ_UNKNOWN = 0;
    protected static final int OFFSETS_LIST_MAX_LENGTH = 60; 
    
    private MobilityECEFLocationHelper startLocationHelper;
    private int[][] offsets;
    private MessageFactory factory;
    
    public MobilityTrajectoryHelper(Trajectory traj) {
        this.setStartLocationHelper(traj.getLocation());
        this.setOffsets(traj.getOffsets());
    }
    
    public MobilityTrajectoryHelper(MessageFactory messageFactory) {
        this.factory = messageFactory;
    }
    
    public void setStartLocationHelper(LocationECEF location) {
        this.startLocationHelper = new MobilityECEFLocationHelper(location);
    }
    
    public void setOffsets(List<LocationOffsetECEF> offsetList) {
        int arraySize = Math.min(offsetList.size(), OFFSETS_LIST_MAX_LENGTH);
        offsets = new int[3][arraySize];
        for(int i = 0; i < arraySize; i++) {
            offsets[0][i] = this.isInRange(offsetList.get(i).getOffsetX()) ? offsetList.get(i).getOffsetX() : 0;
            offsets[1][i] = this.isInRange(offsetList.get(i).getOffsetY()) ? offsetList.get(i).getOffsetY() : 0;
            offsets[2][i] = this.isInRange(offsetList.get(i).getOffsetZ()) ? offsetList.get(i).getOffsetZ() : 0;
        }
    }

    public MobilityECEFLocationHelper getStartLocationHelper() {
        return startLocationHelper;
    }

    public int[][] getOffsets() {
        return offsets;
    }
    
    private boolean isInRange(int data) {
        return data >= XYZ_MIN && data <= XYZ_MAX;
    }
    
    public List<LocationOffsetECEF> intArrayOffsetsToOffsetList(int[][] array) {
        int size = 60;
        for(int i = 0; i < array[0].length; i++) {
            if(array[0][i] == 501 && array[1][i] == 501 && array[2][i] == 501) {
                size = i;
                break;
            }
        }
        List<LocationOffsetECEF> result = new LinkedList<LocationOffsetECEF>();
        for(int i = 0; i < size; i++) {
            LocationOffsetECEF offset = this.factory.newFromType(LocationOffsetECEF._TYPE);
            offset.setOffsetX((short) array[0][i]);
            offset.setOffsetY((short) array[1][i]);
            offset.setOffsetZ((short) array[2][i]);
            result.add(offset);
        }
        return result;
    }
}
