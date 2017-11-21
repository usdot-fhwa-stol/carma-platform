package gov.dot.fhwa.saxton.carma.plugins.speedharm;

import gov.dot.fhwa.saxton.carma.guidance.plugins.AbstractPlugin;
import gov.dot.fhwa.saxton.carma.guidance.plugins.PluginServiceLocator;
import gov.dot.fhwa.saxton.carma.guidance.trajectory.Trajectory;

/**
 * Plugin implementing integration withe STOL I TO 22 Infrastructure Server
 * <p>
 * Commmunicates via the internet with the Infrastructure Server to report vehicle
 * state and receive speed commands as may relate to whatever algorithm the server
 * is configured to run with.
 */
public class SpeedHarmonizationPlugin extends AbstractPlugin{

  public SpeedHarmonizationPlugin(PluginServiceLocator psl) {
    super(psl);
  }

	@Override
	public void onInitialize() {
		// STUB
	}

	@Override
	public void onResume() {
		// STUB
	}

	@Override
	public void loop() throws InterruptedException {
		// STUB
	}

	@Override
	public void onSuspend() {
		// STUB
	}

	@Override
	public void onTerminate() {
		// STUB
  }
  
  @Override
  public void planTrajectory(Trajectory traj, double expectedStartSpeed) {
    // STUB
  }
}