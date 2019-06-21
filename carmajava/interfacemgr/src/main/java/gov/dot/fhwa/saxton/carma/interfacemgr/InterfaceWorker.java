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

import gov.dot.fhwa.saxton.carma.rosutils.AlertSeverity;
import gov.dot.fhwa.saxton.carma.rosutils.SaxtonLogger;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import cav_msgs.DriverStatus;

import gov.dot.fhwa.saxton.carma.rosutils.SaxtonBaseNode;

public class InterfaceWorker {

    protected ArrayList<DriverInfo>         drivers_ = new ArrayList<DriverInfo>();
    protected int                           waitTime_ = 10;  //seconds that we must wait after last driver registered
    protected IInterfaceMgr                 mgr_;
    protected SaxtonLogger                  log_;
    protected long                          startedWaiting_;
    protected long							systemReadyTime_;
    protected AtomicBoolean                 systemOperational_ = new AtomicBoolean(false);
    protected AtomicBoolean                 controllerReady_ = new AtomicBoolean(false);
    protected AtomicBoolean                 gnssReady_ = new AtomicBoolean(false);
    protected AtomicBoolean                 lidarReady_ = new AtomicBoolean(false);

    InterfaceWorker(IInterfaceMgr mgr, SaxtonLogger log) {
        mgr_ = mgr;
        log_ = log;
        startedWaiting_ = System.currentTimeMillis();
    }

    /**
     * Stores the desired wait time for drivers to be discovered before declaring the system operational.
     *
     * NOTE: the wait is only approximate. In fact, it is likely that the elapsed time is truncated to the next
     * lowest whole second before this comparison is made, so the actual wait time could be up to 1 sec more
     * than this threshold.
     *
     * @param wait - number of seconds to wait for new drivers to announce themselves
     */
    public void setWaitTime(int wait) {
        waitTime_ = wait;
        if (log_ != null) {
            log_.debug("STARTUP", "InterfaceWorker: driver wait time set at " + wait + " seconds.");
        }
    }

    /**
     * Updates the info known about a current driver.  Note that this is expected to be called frequently throughout
     * the life of the node.
     * Only Operational drivers are added to the list of drivers available to other nodes.
     * If a driver starts in a degraded state it will only be added once it becomes operational. (TODO Support initially degraded drivers)
     *
     * @param newDriver - all available details about the driver publishing its status
     */
    public void handleNewDriverStatus(DriverInfo newDriverObj) {
        DriverStatus newDriver = newDriverObj.getMsg();
        String name = newDriver.getName();

        //if we already know about this driver then
        int index = getDriverIndex(name);
        if (index >= 0) {

            //if its info has changed then
            if (!newDriverObj.equalCategoryAndState(drivers_.get(index))) {
                log_.debug("DRIVER", "InterfaceWorker.handleNewDriverStatus: status changed for " + name);
                //record the updates. Will need to fetch new driver api as well
                drivers_.set(index, newDriverObj);
                if ((newDriver.getGnss() || newDriver.getController() || newDriver.getLidar())
                  && (newDriver.getStatus() == DriverStatus.FAULT
                  || newDriver.getStatus() == DriverStatus.OFF)) {
                    mgr_.errorShutdown("FAULT detected in critical driver: " + newDriver.getName());
                }
            }
        //else it's a newly discovered driver
        }else {
            // Only operational drivers are considered available for use
            if (newDriver.getStatus() == DriverStatus.OPERATIONAL) {
                drivers_.add(newDriverObj);

                //indicate if this is one of the critical drivers
                if (newDriver.getGnss()) {
                    gnssReady_.set(true);
                }else if (newDriver.getController()) {
                    controllerReady_ .set(true);
                }else if (newDriver.getLidar()) {
                    lidarReady_.set(true);
                }

                //reset the wait timer
                startedWaiting_ = System.currentTimeMillis();

            //else if it is a gnss driver in a degraded state, allow it
            }else if(newDriver.getStatus() == DriverStatus.DEGRADED  &&  newDriver.getGnss()) {
                drivers_.add(newDriverObj);
                gnssReady_.set(true);
                startedWaiting_ = System.currentTimeMillis();
            }

            log_.info("STARTUP", "InterfaceWorker.handleNewDriverStatus: discovered new driver " + name +
                "state = " + newDriver.getStatus());
        }
    }

    /**
     * Indicates whether the system has discovered all the device drivers that it is likely to discover, thus
     * making it ready for operations.
     * Waits an additional amount of time after the latest detected driver in case any further drivers
     * come on line.
     *
     * @return - true if the system is OPERATIONAL
     */
    public boolean isSystemReady() {

        //if system is not yet OPERATIONAL then
        if (!systemOperational_.get()) {

            //if wait timer has expired then
            long elapsed = System.currentTimeMillis() - startedWaiting_;
            if (elapsed > 1000*waitTime_) {

                //if we have the essential drivers registered then
                if (controllerReady_.get()  &&  gnssReady_.get() && lidarReady_.get()) {

                    //indicate that it is now OPERATIONAL
                    systemOperational_.set(true);
                    //log the time required to get to this point
                    log_.info("STARTUP", "///// InterfaceWorker says all known drivers are initialized -- after "
                                + elapsed/1000 + " sec");

                    //record the time of this event
                    systemReadyTime_ = System.currentTimeMillis();

                }else {
                    //log an error and shut down
                    log_.error("DRIVERS", "InterfaceWorker: missing one or more essential drivers - initiating system shutdown.");
                    mgr_.errorShutdown("Unable to discover essential device drivers.  INITIATING SYSTEM SHUTDOWN.");
                }
            }
        }

        return systemOperational_.get();
    }

    /**
     * Returns true if a controller driver has been detected
     * @return true if controller detected
     */
    boolean controllerReady() {
        return controllerReady_.get();
    }
    
    
    /**
     * Returns the time since this class declared the system ready for operation.
     * @return time in ms
     */
    public long timeSinceSystemReady() {
    	return System.currentTimeMillis() - systemReadyTime_;
    }

    //////////

    /**
     * Returns the index in the drivers_ array that matches the name of the given driver.
     *
     * @param givenName - the one we are looking for
     * @return - index of the driver that matches given
     */
    protected int getDriverIndex(String givenName) {

        if (drivers_.size() > 0) {
            for (int i = 0;  i < drivers_.size();  ++i) {
                if (drivers_.get(i).getMsg().getName().equals(givenName)) {
                    return i;
                }
            }
        }

        return -1;
    }
}
