/**
 * This is the worker class that contains all of the logic for the interface manager.
 **/

package gov.dot.fhwa.saxton.carma.interfacemgr;


import java.util.ArrayList;
import java.util.List;

public class InterfaceWorker {

    protected ArrayList<DriverInfo>         drivers_ = new ArrayList<DriverInfo>();
    protected ArrayList<Integer>            numDrivers_ = new ArrayList<Integer>();
    protected int                           waitTime_ = 0;  //seconds that we wait after last driver registered
                                                            // before declaring the system ready to operate

    public InterfaceWorker(IInterfaceMgr mgr) {
    }

    /**
     * Updates the info known about a current driver.  Note that this is expected to be called frequently throughout
     * the life of the node.
     *
     * @param info - all available details about the driver publishing its status
     */
    public void handleNewDriverStatus(DriverInfo info) {
        //if we already know about this driver then
        int index = getDriverIndex(info);
        if (index >= 0) {

            //if its info has changed then
                //record the updates
        //else
        }else {
            //get its list of capabilities (getDriverApi)
            //add the info to the list of known drivers
            //increment the driver type counter
            //request InterfaceMgr to bind with it
            //reset the wait timer
        }
    }

    /**
     * Updates the status of a driver that has broken its bond.  A broken bond simply indicates a status change for
     * that driver; it may still be alive and functioning, but at a different level of capability (it may have even
     * corrected a previous deficiency, e.g. gone from DEGRADED to fully functional).
     *
     * @param driverName - unique ID of the driver
     */
    public void handleBrokenBond(String driverName) {
        //look up the driver and determine its new set of properties, and store them

        //if functionality is totally unavailable then
             //decrement the appropriate driver type counter
             //remove the driver from the list of available drivers

         //if the system is OPERATIONAL then
            //formulate an alert message at the appropriate level depending on the type of driver that is reporting
            // (notifyBrokenBond)
    }

    /**
     * Returns a list of drivers that each provide all of the given capabilities once the system is OPERATIONAL.
     *
     * @param capabilities - a list of capabilities that must be met (inclusive)
     * @return - a list of driver names that meet all the capabilities
     */
    public List<String> getDrivers(DriverCategory cat, List<String> capabilities) {
        //if the system is ready for operation then
            //loop through all known drivers
                //if it matches the requested category then
                    //loop through all requested capabilities
                        //if this driver cannot provide this capability then break out of loop
                    //if the driver is satisfactory then
                        //add the driver to the return list

        return new ArrayList<String>();  //TODO - bogus
    }

    /**
     * Indicates whether the system has just become OPERATIONAL.  If it is either not ready or it has
     * been OPERATIONAL in a previous call then it will return false.
     * Waits an additional amount of time after the latest detected driver in case any further drivers
     * come on line.
     *
     * @return - true if the system is newly OPERATIONAL (one call only will return true)
     */
    public boolean isSystemReady() {
        //if system is not yet OPERATIONAL then
            //if wait timer has expired then
                //indicate that it is now OPERATIONAL
                //log the time required to get to this point
                //return true
            //endif
        //endif

        //return false
        return false;

    }

    //////////

    /**
     * Returns the index in the drivers_ array that matches the name of the given driver.
     *
     * @param given - the one we are looking for
     * @return - index of the driver that matches given
     */
    protected int getDriverIndex(DriverInfo given) {

        if (drivers_.size() > 0) {
            for (int i = 0;  i < drivers_.size();  ++i) {
                if (drivers_.get(i).getName().equals(given.getName())) {
                    return i;
                }
            }
        }

        return -1;
    }

    /**
     * Determines if two driver specifications are identical.
     *
     * @param a - the first driver
     * @param b - the second driver
     * @return - true if they are identical; false otherwise
     */
    protected boolean isSame(DriverInfo a, DriverInfo b) {
        if (!a.getName().equals(b.getName())) {
            return false;
        }
        if (a.getStatus() != b.getStatus()) {
            return false;
        }
        if (a.isCan() != b.isCan()) {
            return false;
        }
        if (a.isComms() != b.isComms()) {
            return false;
        }
        if (a.isController() != b.isController()) {
            return false;
        }
        if (a.isPosition() != b.isPosition()) {
            return false;
        }
        if (a.isSensor() != b.isSensor()) {
            return false;
        }

        return true;
    }
}
