package gov.dot.fhwa.saxton.carma.interfacemgr;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import static org.junit.Assert.*;

public class InterfaceWorkerTest {

    IInterfaceMgr   mgr_;
    InterfaceWorker w_;

    @Before
    public void setUp() throws Exception {

        mgr_ = new FakeInterfaceMgr();
        w_ = new InterfaceWorker(mgr_);
    }

    @After
    public void tearDown() throws Exception {
    }

    @Test
    public void handleNewDriverStatus() throws Exception {
    }

    @Test
    public void handleBrokenBond() throws Exception {
    }

    @Test
    public void getDrivers() throws Exception {
    }

    @Test
    public void isSystemReady() throws Exception {
    }

    private void addDrivers() {

    }
}