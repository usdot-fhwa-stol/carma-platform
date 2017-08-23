package gov.dot.fhwa.saxton.carma.interfacemgr;

import java.util.List;

public interface IInterfaceMgr {

    /**
     * Binds with the specified driver node.
     *
     * @param driverName - name of the driver's bind topic
     */
    public void bindWithDriver(String driverName);


    /**
     * Requests the given driver's specific list of data capabilities.
     *
     * @param driverName - name of the driver's api topic
     * @return - a list of data elements available from the driver
     */
    public List<String> getDriverApi(String driverName);


    /**
     * Handler for a detected broken driver bond - sends an appropriate system alert message.
     * Note that this is not the callback to be provided to the driver's bind service.
     *
     * @param sev - severity of the problem
     * @param message - description of the problem
     */
    public void notifyBrokenBond(AlertSeverity sev, String message);
}
