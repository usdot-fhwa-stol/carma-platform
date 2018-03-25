package gov.dot.fhwa.saxton.carma.plugins.platooning;

import cav_msgs.MobilityOperation;
import cav_msgs.MobilityRequest;
import cav_msgs.MobilityResponse;
import gov.dot.fhwa.saxton.carma.guidance.arbitrator.TrajectoryPlanningResponse;
import gov.dot.fhwa.saxton.carma.guidance.mobilityrouter.MobilityRequestResponse;
import gov.dot.fhwa.saxton.carma.guidance.plugins.PluginServiceLocator;
import gov.dot.fhwa.saxton.carma.guidance.trajectory.Trajectory;
import gov.dot.fhwa.saxton.carma.guidance.util.ILogger;

public class CandidateFollowerState implements IPlatooningState {

    protected PlatooningPlugin     plugin;
    protected ILogger              log;
    protected PluginServiceLocator pluginServiceLocator;

    public CandidateFollowerState(PlatooningPlugin plugin, ILogger log, PluginServiceLocator pluginServiceLocator) {
        this.plugin = plugin;
        this.log = log;
        this.pluginServiceLocator = pluginServiceLocator;
    }

    @Override
    public TrajectoryPlanningResponse planTrajectory(Trajectory traj, double expectedEntrySpeed) {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public MobilityRequestResponse onMobilityRequestMessgae(MobilityRequest msg) {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public void onMobilityOperationMessage(MobilityOperation msg) {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void onMobilityResponseMessage(MobilityResponse msg) {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void run() {
        // TODO Auto-generated method stub
        
    }
    
}
