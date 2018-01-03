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

package gov.dot.fhwa.saxton.carma.interfacemgr;

import gov.dot.fhwa.saxton.carma.rosutils.SaxtonLogger;
import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import org.ros.message.MessageFactory;
import org.ros.node.NodeConfiguration;
import java.util.ArrayList;
import java.util.List;

import static org.junit.Assert.*;

public class LateralControlDriverTest {

    ILateralControlDriver driver_;
    LateralControlWorker w_;
    SaxtonLogger log_;
    NodeConfiguration nodeConfiguration = NodeConfiguration.newPrivate();
    MessageFactory messageFactory = nodeConfiguration.getTopicMessageFactory();

    @Before
    public void setUp() throws Exception {
        log_ = new SaxtonLogger(this.class.getSimpleName(), LogFactory.getLog(InterfaceWorkerTest.class));
        driver_ = new FakeLateralControlDriver();
        w_ = new LateralControlWorker(driver_, log_);
    }

    @Test
    public void testUIInstructions() throws Exception {
        log_.info("///// Entering testUIInstructions.");
        cav_msgs.LateralControl msg = messageFactory.newFromType(cav_msgs.LateralControl._TYPE);
        msg.setAxelAngle(0.0);
        w_.handleLateralControlMsg(msg);
        assertFalse(driver_.getRightTurnMsgReceived());
        assertFalse(driver_.getLeftTurnMsgReceived());

        msg.setAxelAngle(2.0);
        w_.handleLateralControlMsg(msg);
        assertFalse(driver_.getRightTurnMsgReceived());
        assertFalse(driver_.getLeftTurnMsgReceived());

        msg.setAxelAngle(-2.0);
        w_.handleLateralControlMsg(msg);
        assertFalse(driver_.getRightTurnMsgReceived());
        assertFalse(driver_.getLeftTurnMsgReceived());

        msg.setAxelAngle(-2.1);
        w_.handleLateralControlMsg(msg);
        assertFalse(driver_.getRightTurnMsgReceived());
        assertTrue(driver_.getLeftTurnMsgReceived());

        driver_.resetFlags();

        msg.setAxelAngle(2.1);
        w_.handleLateralControlMsg(msg);
        assertTrue(driver_.getRightTurnMsgReceived());
        assertFalse(driver_.getLeftTurnMsgReceived());
    }
}