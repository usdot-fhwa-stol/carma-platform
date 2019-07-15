/*
 * Copyright (C) 2018-2019 LEIDOS.
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

package gov.dot.fhwa.saxton.carma.interfacemgr;

import java.util.ArrayList;
import java.util.List;

import org.ros.message.MessageFactory;
import org.ros.node.NodeConfiguration;

public class DriverInfo {

    cav_msgs.DriverStatus msg;

    public DriverInfo() {
        final MessageFactory messageFactory = NodeConfiguration.newPrivate().getTopicMessageFactory();
        msg = messageFactory.newFromType(cav_msgs.DriverStatus._TYPE);
    }

    public cav_msgs.DriverStatus getMsg() {
        return msg;
    }

    public void setMsg(cav_msgs.DriverStatus msg) {
        this.msg = msg;
    }

    public boolean equalCategoryAndState(DriverInfo b) {
        if (!msg.getName().equals(b.getMsg().getName())) {
            return false;
        }
        if (msg.getStatus() != b.getMsg().getStatus()) {
            return false;
        }
        if (msg.getCan() != (b.getMsg().getCan())) {
            return false;
        }
        if (msg.getRadar() != b.getMsg().getRadar()) {
            return false;
        }
        if (msg.getLidar() != b.getMsg().getLidar()) {
            return false;
        }
        if (msg.getRoadwaySensor() != b.getMsg().getRoadwaySensor()) {
            return false;
        }
        if (msg.getComms() != b.getMsg().getComms()) {
            return false;
        }
        if (msg.getController() != b.getMsg().getController()) {
            return false;
        }
        if (msg.getCamera() != b.getMsg().getCamera()) {
            return false;
        }
        if (msg.getImu() != b.getMsg().getImu()) {
            return false;
        }
        

        return true;
    }
}
