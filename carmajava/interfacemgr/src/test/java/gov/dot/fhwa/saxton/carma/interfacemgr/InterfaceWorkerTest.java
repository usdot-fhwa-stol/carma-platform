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

package gov.dot.fhwa.saxton.carma.interfacemgr;

import gov.dot.fhwa.saxton.carma.rosutils.SaxtonLogger;
import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

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
    public void testGettingEmptyDrivers() throws Exception {
        log_.info("///// Entering testGettingEmptyDrivers.");

        //build a list of capabilities we're looking for
        List<String> capabilities = new ArrayList<String>();
        capabilities.add("latitude");
        capabilities.add("longitude");
        capabilities.add("elevation");

        //since there are no drivers specified yet, the system should not be considered
        // OPERATIONAL, so this should return an empty list
        List<String> res = w_.getDrivers(capabilities);
        assertEquals(res.size(), 0);
    }

    @Test
    public void testGetDriversSimple() throws Exception {
        log_.info("///// Entering testGetDriversSimple.");

        w_.setWaitTime(1); //1 sec wait time for system to become active

        //build a list of capabilities we're looking for
        List<String> capabilities = new ArrayList<String>();
        capabilities.add("latitude");
        capabilities.add("longitude");
        capabilities.add("elevation");

        //add some drivers
        addNewDrivers();
        addControllerDriver();

        //wait for the system to become operational
        Thread.sleep(2005);
        boolean ready = w_.isSystemReady(); //need to make this call to force the flag to change since the FakeInterfaceMgr isn't monitoring
        assertTrue(ready);

        //let's go find them
        List<String> res = w_.getDrivers(capabilities);
        assertEquals(res.size(), 3);
        assertEquals(res.get(0), "position3/latitude");

        //simplify the list of capabilities we need
        capabilities.clear();
        res.clear();
        assertEquals(res.size(), 0);

        capabilities.add("longitude");
        capabilities.add("latitude"); //purposely did these in reverse order
        res = w_.getDrivers(capabilities);
        assertEquals(res.size(), 4);
        assertEquals(res.get(0), "position1/longitude");
        assertEquals(res.get(2), "position3/longitude");
        assertEquals(res.get(3), "position3/latitude");

        //look for another capability
        capabilities.clear();
        res.clear();
        assertEquals(res.size(), 0);

        capabilities.add("acceleration");
        res = w_.getDrivers(capabilities);
        assertEquals(res.size(), 1);
        assertEquals(res.get(0), "position3/acceleration");
    }

    @Test
    public void testGetDriversCategories() throws Exception {
        log_.info("///// Entering testGetDriversCategories.");

        w_.setWaitTime(1); //1 sec wait time for system to become active

        //build a list of capabilities we're looking for
        List<String> capabilities = new ArrayList<String>();
        List<String> res;

        //create new drivers
        DriverInfo position4 = new DriverInfo();
        position4.setPosition(true);
        position4.setName("~/pinpoint/position/position4"); //provides lat/lon/elev w/long name
        position4.setState(DriverState.OPERATIONAL);
        w_.handleNewDriverStatus(position4);

        DriverInfo position5 = new DriverInfo();
        position5.setPosition(true);
        position5.setName("position/position5"); //provides lat/lon/elev w/position category in name
        position5.setState(DriverState.OPERATIONAL);
        w_.handleNewDriverStatus(position5);

        DriverInfo sensor5 = new DriverInfo();
        sensor5.setSensor(true);
        sensor5.setName("radar/typeR/sensor/R5"); //provides elev only; long name
        sensor5.setState(DriverState.OPERATIONAL);
        w_.handleNewDriverStatus(sensor5);

        addControllerDriver();

        //wait for the system to become operational
        Thread.sleep(2005);
        boolean ready = w_.isSystemReady(); //need to make this call to force the flag to change since the FakeInterfaceMgr isn't monitoring
        assertTrue(ready);

        //look for lat/lon from a position driver
        capabilities.add("position/latitude");
        capabilities.add("position/longitude");
        res = w_.getDrivers(capabilities);
        assertEquals(res.size(), 4);
        assertEquals(res.get(1), "~/pinpoint/position/position4/longitude");
        assertEquals(res.get(2), "position/position5/latitude");

        //look for elevation from a position driver
        capabilities.clear();
        capabilities.add("position/elevation");
        res = w_.getDrivers(capabilities);
        assertEquals(res.size(), 2);
        assertTrue(res.get(0).equals("~/pinpoint/position/position4/elevation"));
        assertTrue(res.get(1).equals("position/position5/elevation"));

        //look for elevation from any type of driver
        capabilities.clear();
        capabilities.add("elevation");
        res = w_.getDrivers(capabilities);
        assertEquals(res.size(), 3);
        assertTrue(res.get(2).equals("radar/typeR/sensor/R5/elevation"));
    }

    @Test
    public void testSystemNotReady() throws Exception {
        log_.info("///// Entering testSystemNotReady.");

        //system just started up, no drivers registered, so should not be ready
        boolean res = w_.isSystemReady();
        assertFalse(res);
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
        Thread.sleep(1000); //1.0 sec
        ready = w_.isSystemReady();
        assertFalse(ready);

        //add some more updates - this will not reset the timer since there are no new drivers being discovered
        addDuplicateDrivers();
        continueStatusUpdates();
        Thread.sleep(900); //1.9 sec
        ready = w_.isSystemReady();
        assertFalse(ready);

        Thread.sleep(2500); //4.4 sec
        ready = w_.isSystemReady();
        assertFalse(ready);

        Thread.sleep(1650); //6.05 sec - apparently the comparison truncates fractional seconds so we need this much
        ready = w_.isSystemReady();
        assertTrue(ready);
        assertFalse(mgr_.isShutdownUnderway());
    }

    @Test (expected = IndexOutOfBoundsException.class)
    public void handleBrokenBond() throws Exception {
        log_.info("///// Entering handleBrokenBond.");

        w_.setWaitTime(1); //1 second
        boolean ready = w_.isSystemReady(); //system should not be ready yet
        assertFalse(ready);

        //set up a list of capabilities that all drivers will match (a null list)
        List<String> capabilities = new ArrayList<String>();

        //set up drivers and get their bonds set up
        addNewDrivers();
        addDuplicateDrivers(); //this will set sensor1 to FAULT

        //at this point the system will not be OPERATIONAL because not enough time has elapsed
        w_.handleBrokenBond("sensor/sensor1");

        //since sensor1 had a FAULT it should not appear in the list of available drivers
        List<String> res = w_.getDrivers(capabilities);
        assertEquals(res.size(), 0); //three POSITION drivers only, which will not be returned

        //wait until system becomes OPERATIONAL
        Thread.sleep(2005);
        ready = w_.isSystemReady(); //need to make this call to force the flag to change since the FakeInterfaceMgr isn't monitoring

        w_.handleBrokenBond("sensor1"); //this should write a message to the log AND THROW AN EXCEPTION
    }

    @After
    public void tearDown() throws Exception {
    }


    //////////


    private void addNewDrivers() {
        //Two drivers of same type, but no duplications here

        DriverInfo position1 = new DriverInfo();
        position1.setPosition(true);
        position1.setName("position1");
        position1.setState(DriverState.OPERATIONAL);
        w_.handleNewDriverStatus(position1);

        DriverInfo position2 = new DriverInfo();
        position2.setPosition(true);
        position2.setName("position2");
        position2.setState(DriverState.FAULT);
        w_.handleNewDriverStatus(position2);

        DriverInfo position3 = new DriverInfo();
        position3.setPosition(true);
        position3.setName("position3");
        position3.setState(DriverState.OPERATIONAL);
        w_.handleNewDriverStatus(position3);

        DriverInfo sensor1 = new DriverInfo();
        sensor1.setSensor(true);
        sensor1.setName("sensor1");
        sensor1.setState(DriverState.OPERATIONAL);
        w_.handleNewDriverStatus(sensor1);

    }

    //each driver will publish status updates periodically, so we should expect to see
    // lots of repeats of the same status info coming in

    private void addDuplicateDrivers() {

        //duplicate the sensor1 from above with same OPERATIONAL status
        DriverInfo sensor1 = new DriverInfo();
        sensor1.setSensor(true);
        sensor1.setName("sensor1");
        sensor1.setState(DriverState.OPERATIONAL);
        w_.handleNewDriverStatus(sensor1);

        //same driver again with a different status
        sensor1.setState(DriverState.FAULT);
        w_.handleNewDriverStatus(sensor1);
    }

    private void addControllerDriver() {
        DriverInfo controllerDriver = new DriverInfo();
        controllerDriver.setController(true);
        controllerDriver.setName("controller1");
        controllerDriver.setState(DriverState.OPERATIONAL);
        w_.handleNewDriverStatus(controllerDriver);
    }

    private void continueStatusUpdates() {
        DriverInfo sensor1 = new DriverInfo();
        sensor1.setSensor(true);
        sensor1.setName("sensor1");
        sensor1.setState(DriverState.OPERATIONAL);
        w_.handleNewDriverStatus(sensor1);

        DriverInfo position3 = new DriverInfo();
        position3.setPosition(true);
        position3.setName("position3");
        position3.setState(DriverState.OPERATIONAL);
        w_.handleNewDriverStatus(position3);

        w_.handleNewDriverStatus(sensor1);
        w_.handleNewDriverStatus(position3);
    }
}