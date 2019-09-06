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

import gov.dot.fhwa.saxton.carma.rosutils.SaxtonLogger;
import org.apache.commons.logging.LogFactory;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import cav_msgs.DriverStatus;

import java.util.ArrayList;
import java.util.List;

import static org.junit.Assert.*;

public class InterfaceWorkerTest {

    IInterfaceMgr   mgr_;
    InterfaceWorker w_;
    SaxtonLogger    log_;

    @Before
    public void setUp() throws Exception {
        log_ = new SaxtonLogger(InterfaceWorkerTest.class.getSimpleName(), LogFactory.getLog(InterfaceWorkerTest.class));
        mgr_ = new FakeInterfaceMgr();
        w_ = new InterfaceWorker(mgr_, log_);
    }

    @Test
    public void testNewDrivers() throws Exception {
        log_.info("///// Entering testNewDrivers.");
        addNewDrivers();
    }

    @Test
    public void testDuplicateDriver() {
        log_.info("///// Entering testDuplicateDriver");
        addDuplicateDrivers();
    }

    @Test
    public void testSystemNotReady() throws Exception {
        log_.info("///// Entering testSystemNotReady.");

        //system just started up, no drivers registered, so should not be ready
        boolean res = w_.isSystemReady();
        assertFalse(res);
    }

    @Test
    public void testSystemReadyNoLidar() throws Exception {
        log_.info("///// Entering testSystemReady.");

        w_.setWaitTime(5); //5 seconds
        boolean ready = w_.isSystemReady();
        assertFalse(ready);

        //initial driver discovery
        addNewDrivers();
        Thread.sleep(1000); //1.0 sec
        ready = w_.isSystemReady();
        assertFalse(ready);

        //add some more updates - this will not reset the timer since there are no new drivers being discovered
        addDuplicateDrivers();
        continueStatusUpdates();
        addControllerDriver();
        Thread.sleep(900); //1.9 sec
        ready = w_.isSystemReady();
        assertFalse(ready);

        //one more update
        continueStatusUpdates();
        Thread.sleep(1500); //3.4 sec
        ready = w_.isSystemReady();
        assertFalse(ready);

        Thread.sleep(1000); //4.4 sec
        ready = w_.isSystemReady();
        assertFalse(ready);

        //this one will fail also, even though time has expired, because the lidar driver was never registered
        Thread.sleep(1605); //6.05 sec - apparently the comparison truncates fractional seconds so we need this much
        ready = w_.isSystemReady();
        assertFalse(ready);
        assertTrue(mgr_.isShutdownUnderway());
    }

    @Test
    public void testSystemReadyNoController() throws Exception {
        log_.info("///// Entering testSystemReady.");

        w_.setWaitTime(5); //5 seconds
        boolean ready = w_.isSystemReady();
        assertFalse(ready);

        //initial driver discovery
        addNewDrivers();
        Thread.sleep(1000); //1.0 sec
        ready = w_.isSystemReady();
        assertFalse(ready);

        //add some more updates - this will not reset the timer since there are no new drivers being discovered
        addDuplicateDrivers();
        continueStatusUpdates();
        addLidarDriver();
        Thread.sleep(900); //1.9 sec
        ready = w_.isSystemReady();
        assertFalse(ready);

        //one more update
        continueStatusUpdates();
        Thread.sleep(1500); //3.4 sec
        ready = w_.isSystemReady();
        assertFalse(ready);

        Thread.sleep(1000); //4.4 sec
        ready = w_.isSystemReady();
        assertFalse(ready);

        //this one will fail also, even though time has expired, because the controller driver was never registered
        Thread.sleep(1605); //6.05 sec - apparently the comparison truncates fractional seconds so we need this much
        ready = w_.isSystemReady();
        assertFalse(ready);
        assertTrue(mgr_.isShutdownUnderway());
    }

    @Test
    public void testSystemReadySuccess() throws Exception {
        log_.info("///// Entering testSystemReady.");

        w_.setWaitTime(5); //5 seconds
        boolean ready = w_.isSystemReady();
        assertFalse(ready);

        //initial driver discovery
        addNewDrivers();
        addControllerDriver();
        addLidarDriver();
        Thread.sleep(1000); //1.0 sec
        ready = w_.isSystemReady();
        assertFalse(ready);

        //add some more updates - this will not reset the timer since there are no new drivers being discovered
        addDuplicateDrivers();
        continueStatusUpdates();
        addControllerDriver();
        addLidarDriver();
        Thread.sleep(900); //1.9 sec
        ready = w_.isSystemReady();
        assertFalse(ready);

        Thread.sleep(2500); //4.4 sec
        ready = w_.isSystemReady();
        addControllerDriver();
        addLidarDriver();
        continueStatusUpdates();
        assertFalse(ready);

        Thread.sleep(1650); //6.05 sec - apparently the comparison truncates fractional seconds so we need this much
        continueStatusUpdates();
        addControllerDriver();
        addLidarDriver();
        ready = w_.isSystemReady();
        assertTrue(ready);
        assertFalse(mgr_.isShutdownUnderway());
    }

    @After
    public void tearDown() throws Exception {
    }


    //////////


    private void addNewDrivers() {
        //Two drivers of same type, but no duplications here

        DriverInfo position1 = new DriverInfo();
        position1.getMsg().setGnss(true);
        position1.getMsg().setName("position1");
        position1.getMsg().setStatus(DriverStatus.OPERATIONAL);
        w_.handleNewDriverStatus(position1);

        DriverInfo position2 = new DriverInfo();
        position2.getMsg().setGnss(true);
        position2.getMsg().setName("position2");
        position2.getMsg().setStatus(DriverStatus.FAULT);
        w_.handleNewDriverStatus(position2);

        DriverInfo position3 = new DriverInfo();
        position3.getMsg().setGnss(true);
        position3.getMsg().setName("position3");
        position3.getMsg().setStatus(DriverStatus.OPERATIONAL);
        w_.handleNewDriverStatus(position3);

        DriverInfo sensor1 = new DriverInfo();
        sensor1.getMsg().setCan(true);
        sensor1.getMsg().setName("sensor1");
        sensor1.getMsg().setStatus(DriverStatus.OPERATIONAL);
        w_.handleNewDriverStatus(sensor1);

    }

    //each driver will publish status updates periodically, so we should expect to see
    // lots of repeats of the same status info coming in

    private void addDuplicateDrivers() {

        //duplicate the sensor1 from above with same OPERATIONAL status
        DriverInfo sensor1 = new DriverInfo();
        sensor1.getMsg().setCan(true);
        sensor1.getMsg().setName("sensor1");
        sensor1.getMsg().setStatus(DriverStatus.OPERATIONAL);
        w_.handleNewDriverStatus(sensor1);

        //same driver again with a different status
        sensor1.getMsg().setStatus(DriverStatus.FAULT);
        w_.handleNewDriverStatus(sensor1);
    }

    private void addControllerDriver() {
        DriverInfo controllerDriver = new DriverInfo();
        controllerDriver.getMsg().setController(true);
        controllerDriver.getMsg().setName("controller1");
        controllerDriver.getMsg().setStatus(DriverStatus.OPERATIONAL);
        w_.handleNewDriverStatus(controllerDriver);
    }

    private void addLidarDriver() {
        DriverInfo lidarDriver = new DriverInfo();
        lidarDriver.getMsg().setLidar(true);
        lidarDriver.getMsg().setName("lidar1");
        lidarDriver.getMsg().setStatus(DriverStatus.OPERATIONAL);
        w_.handleNewDriverStatus(lidarDriver);
    }

    private void continueStatusUpdates() {
        DriverInfo sensor1 = new DriverInfo();
        sensor1.getMsg().setCan(true);
        sensor1.getMsg().setName("sensor1");
        sensor1.getMsg().setStatus(DriverStatus.OPERATIONAL);
        w_.handleNewDriverStatus(sensor1);

        DriverInfo position3 = new DriverInfo();
        position3.getMsg().setGnss(true);
        position3.getMsg().setName("position3");
        position3.getMsg().setStatus(DriverStatus.OPERATIONAL);
        w_.handleNewDriverStatus(position3);

        w_.handleNewDriverStatus(sensor1);
        w_.handleNewDriverStatus(position3);
    }
}