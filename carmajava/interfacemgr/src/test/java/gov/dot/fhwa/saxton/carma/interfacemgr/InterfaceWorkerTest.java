/*
 * TODO: Copyright (C) 2017 LEIDOS.
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
 **/

package gov.dot.fhwa.saxton.carma.interfacemgr;

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

    @Before
    public void setUp() throws Exception {
        Log log_ = LogFactory.getLog(InterfaceWorkerTest.class);
        mgr_ = new FakeInterfaceMgr();
        w_ = new InterfaceWorker(mgr_, log_);
    }

    @Test
    public void testNewDrivers() throws Exception {
        addNewDrivers();
    }

    @Test
    public void testDuplicateDriver() {
        addDuplicateDrivers();
    }

   @Test
    public void testGettingEmptyDrivers() throws Exception {
        //build a list of capabilities we're looking for
        List<String> capabilities = new ArrayList<String>();
        capabilities.add("latitude");
        capabilities.add("longitude");
        capabilities.add("elevation");

        //since there are no drivers specified yet, the system should not be considered
        // OPERATIONAL, so this should return an empty list
        List<String> res = w_.getDrivers(DriverCategory.POSITION, capabilities);
        assertEquals(res.size(), 0);
    }

    @Test
    public void testGetDrivers() throws Exception {
        //build a list of capabilities we're looking for
        List<String> capabilities = new ArrayList<String>();
        capabilities.add("latitude");
        capabilities.add("longitude");
        capabilities.add("elevation");

        //add some drivers
        addNewDrivers();

        //let's go find them
        List<String> res = w_.getDrivers(DriverCategory.POSITION, capabilities);
        assertEquals(res.size(), 1);
        assertEquals(res.get(0), "position3");

        //simplify the list of capabilities we need
        capabilities.clear();
        res.clear();
        assertEquals(res.size(), 0);

        capabilities.add("longitude");
        capabilities.add("latitude"); //purposely did these in reverse order
        res = w_.getDrivers(DriverCategory.POSITION, capabilities);
        assertEquals(res.size(), 2);
        assertEquals(res.get(0), "position1");
        assertEquals(res.get(1), "position3");

        //look for another capability
        capabilities.clear();
        res.clear();
        assertEquals(res.size(), 0);

        capabilities.add("acceleration");
        res = w_.getDrivers(DriverCategory.POSITION, capabilities);
        assertEquals(res.size(), 1);
        assertEquals(res.get(0), "position3");
    }

    @Test
    public void testSystemNotReady() throws Exception {
        //system just started up, no drivers registered, so should not be ready
        boolean res = w_.isSystemReady();
        assertFalse(res);
    }

    @Test
    public void testSystemReady() throws Exception {
        //initial driver discovery
        addNewDrivers();
        Thread.sleep(1853);
        assertFalse(w_.isSystemReady());

        //NOTE: assumes total wait time is 5 seconds

        //add some more updates
        addDuplicateDrivers();
        continueStatusUpdates();
        Thread.sleep(2900);
        assertFalse(w_.isSystemReady());

        //one more update
        continueStatusUpdates();
        Thread.sleep(1500);
        assertFalse(w_.isSystemReady());

        //sleep 2 more seconds for total of 3.5 sec
        Thread.sleep(2000);
        assertFalse(w_.isSystemReady());

        //sleep 1.2 more seconds, total of 4.7 sec
        Thread.sleep(1200);
        assertFalse(w_.isSystemReady());

        //one more update - this should reset the wait counter to zero
        continueStatusUpdates();
        Thread.sleep(1000);
        assertFalse(w_.isSystemReady());

        //sleep 4.1 more seconds, which should expire the timer (total 5.1 sec)
        Thread.sleep(4100);
        assertTrue(w_.isSystemReady());
    }

    @Test
    public void handleBrokenBond() throws Exception {

        //set up a list of capabilities that all drivers will match (a null list)
        List<String> capabilities = new ArrayList<String>();

        //set up drivers and get their bonds set up
        addNewDrivers();
        addDuplicateDrivers(); //this will set sensor1 to FAULT

        //at this point the system will not be OPERATIONAL because not enough time has elapsed
        w_.handleBrokenBond("sensor1");

        //since sensor1 had a FAULT it should not appear in the list of available drivers
        List<String> res = w_.getDrivers(DriverCategory.SENSOR, capabilities);
        assertEquals(res.size(), 3); //three POSITION drivers only
        for (String s : res) {
            assertNotEquals(s, "sensor1");
        }

        //wait until system becomes OPERATIONAL
        Thread.sleep(5005);

        w_.handleBrokenBond("sensor1"); //this should write a message to the log
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
        position1.setStatus(DriverState.OPERATIONAL);
        w_.handleNewDriverStatus(position1);

        DriverInfo position2 = new DriverInfo();
        position2.setPosition(true);
        position2.setName("position2");
        position2.setStatus(DriverState.FAULT);
        w_.handleNewDriverStatus(position2);

        DriverInfo position3 = new DriverInfo();
        position3.setPosition(true);
        position3.setName("position3");
        position3.setStatus(DriverState.OPERATIONAL);
        w_.handleNewDriverStatus(position3);

        DriverInfo sensor1 = new DriverInfo();
        sensor1.setSensor(true);
        sensor1.setName("sensor1");
        sensor1.setStatus(DriverState.OPERATIONAL);
        w_.handleNewDriverStatus(sensor1);

    }

    //each driver will publish status updates periodically, so we should expect to see
    // lots of repeats of the same status info coming in

    private void addDuplicateDrivers() {

        //duplicate the sensor1 from above with same OPERATIONAL status
        DriverInfo sensor1 = new DriverInfo();
        sensor1.setSensor(true);
        sensor1.setName("sensor1");
        sensor1.setStatus(DriverState.OPERATIONAL);
        w_.handleNewDriverStatus(sensor1);

        //same driver again with a different status
        sensor1.setStatus(DriverState.FAULT);
        w_.handleNewDriverStatus(sensor1);
    }

    private void continueStatusUpdates() {
        DriverInfo sensor1 = new DriverInfo();
        sensor1.setSensor(true);
        sensor1.setName("sensor1");
        sensor1.setStatus(DriverState.OPERATIONAL);
        w_.handleNewDriverStatus(sensor1);

        DriverInfo position3 = new DriverInfo();
        position3.setPosition(true);
        position3.setName("position3");
        position3.setStatus(DriverState.OPERATIONAL);
        w_.handleNewDriverStatus(position3);

        w_.handleNewDriverStatus(sensor1);
        w_.handleNewDriverStatus(position3);
    }
}