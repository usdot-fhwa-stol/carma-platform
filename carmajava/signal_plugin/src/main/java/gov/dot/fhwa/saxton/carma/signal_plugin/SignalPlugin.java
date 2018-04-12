package gov.dot.fhwa.saxton.carma.signal_plugin;

import gov.dot.fhwa.saxton.carma.guidance.arbitrator.TrajectoryPlanningResponse;
import gov.dot.fhwa.saxton.carma.guidance.plugins.AbstractPlugin;
import gov.dot.fhwa.saxton.carma.guidance.plugins.IStrategicPlugin;
import gov.dot.fhwa.saxton.carma.guidance.plugins.PluginServiceLocator;
import gov.dot.fhwa.saxton.carma.guidance.trajectory.Trajectory;

/**
 * Top level class in the Signal Plugin that does trajectory planning through signalized intersections.
 * This is a port of the functionality developed under the STOL I contract TO 17 and STOL II contract
 * TO 13, Glidepath project.
 */
public class SignalPlugin extends AbstractPlugin implements IStrategicPlugin {

    public SignalPlugin(PluginServiceLocator psl) {
        super(psl);
        version.setName("Signal Plugin");
        version.setMajorRevision(1);
        version.setIntermediateRevision(0);
        version.setMinorRevision(0);
    }

    //TODO: this is a skeleton!  Unless otherwise noted, all comments below are PDL that need to be expanded to working code


    @Override
    public void onInitialize() {
        //load params

        log.info("STARTUP", "SignalPlugin has been initialize.");
        //log the key params here
    }


    @Override
    public void onResume() {

        log.info("SignalPlugin has resumed.");
    }


    @Override
    public void loop() throws InterruptedException {
        long tsStart = System.currentTimeMillis();






        long tsEnd = System.currentTimeMillis();
        long sleepDuration = Math.max(100 - (tsEnd - tsStart), 0);
        Thread.sleep(sleepDuration);
    }


    @Override
    public void onSuspend() {

        log.info("SignalPlugin has been suspended.");
    }


    @Override
    public void onTerminate() {

    }


    @Override
    public TrajectoryPlanningResponse planTrajectory(Trajectory traj, double expectedStartSpeed) {





        return new TrajectoryPlanningResponse();
    }
}
