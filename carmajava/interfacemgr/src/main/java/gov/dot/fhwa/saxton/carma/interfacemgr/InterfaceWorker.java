/**
 * This is the worker class that contains all of the logic for the interface manager.
 **/

package gov.dot.fhwa.saxton.carma.interfacemgr;


import java.util.List;

public class InterfaceWorker {

    public InterfaceWorker(IInterfaceMgr mgr) {

        //initialize counts of each driver type and driver list
        //initialize the wait timer
   }

    /**
     * Updates the info known about a current driver.  Note that this is expected to be called frequently throughout
     * the life of the node.
     *
     * @param info - all available details about the driver publishing its status
     */
    public void handleNewDriverStatus(DriverInfo info) {
        //if we already know about this driver then
            //if its info has changed then
                //record the updates
        //else
            //get its list of capabilities (getDriverApi)
            //add the info to the list of known drivers
            //increment the driver type counter
            //request InterfaceMgr to bind with it
            //reset the wait timer
    }

    /**
     * Updates the status of a driver that has broken its bond.  A broken bond simply indicates a status change for
     * that driver; it may still be alive and functioning, but at a different level of capability (it may have even
     * corrected a previous deficiency, e.g. gone from degraded to fully functional).
     *
     * @param driverId - unique ID
     */
    public void handleBrokenBond(int driverId) {
        //look up the driver and determine its new set of properties, and store them

        //if the system is operational then
            //formulate an alert message at the appropriate level depending on the type of driver that is reporting

        //if functionality is totally unavailable then
            //decrement the appropriate driver type counter
            //remove the driver from the list of available drivers
    }

    /**
     * Returns a list of drivers that each provide all of the given capabilities once the system is operational.
     *
     * @param capabilities - a list of capabilities that must be met (inclusive)
     * @return - a list of driver names that meet all the capabilities
     */
    public List<String> getDrivers(List<String> capabilities) {
        //if the system is ready for operation then
            //loop through all known drivers
                //loop through all requested capabilities
                    //if this driver cannot provide this capability then break out of loop
                //if the driver is satisfactory then
                    //add the driver to the return list

    }

    /**
     * Indicates whether the system has just become operational.  If it is either not ready or it has
     * been operational in a previous call then it will return false.
     * Waits an additional amount of time after the latest detected driver in case any further drivers
     * come on line.
     *
     * @return - true if the system is newly operational (one call only will return true)
     */
    public boolean isSystemReady() {
        //if system is not yet operational then
            //if wait timer has expired then
                //indicate that it is now operational
                //log the time required to get to this point
                //return true
            //endif
        //endif

        //return false
    }
}
