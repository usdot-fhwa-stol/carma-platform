package gov.dot.fhwa.saxton.carma.guidance.lanechange;

import gov.dot.fhwa.saxton.carma.guidance.plugins.AbstractPlugin;
import gov.dot.fhwa.saxton.carma.guidance.plugins.ITacticalPlugin;
import gov.dot.fhwa.saxton.carma.guidance.plugins.PluginServiceLocator;
import gov.dot.fhwa.saxton.carma.guidance.trajectory.Trajectory;

/**
 * This is a mandatory plugin for the Carma platform that manages all lane change activity within a given sub-trajectory.
 * It will create a FutureManeuver place holder for both lateral and longitudinal dimensions, which allows the parent
 * strategic plugin to continue planning the remainder of the trajectory around this space. In parallel to that activity,
 * this plugin will plan the details of the lane change, inserting one or more maneuvers into the FutureManeuver space
 * as necessary to accomplish the mission. This process will involve determining if neighbor vehicles are in the
 * desired target lane space, and, if so, negotiating a coordinated movement with one or more of them to ensure that
 * enough space is opened up in the target lane. Because this negotiation could take significant time, it will be
 * allowed to run in parallel with the planning of the rest of the trajectory (by the parent plugin), and, in fact,
 * can proceed even during execution of the early part of that trajectory, right up to the point that the contents of
 * this resultant FutureManeuver needs to be executed. At that time, if its planning is incomplete an exception will te
 * thrown and that trajectory will be aborted by the parent.
 */

public class LaneChangePlugin extends AbstractPlugin implements ITacticalPlugin {

    public LaneChangePlugin(PluginServiceLocator psl) {
        super(psl);
        version.setName("Lane Change Plugin");
        version.setMajorRevision(1);
        version.setIntermediateRevision(0);
        version.setMinorRevision(0);
    }


    @Override
    public void onInitialize() {
        //TODO - fill in
    }


    @Override
    public void onResume() {
        //TODO - fill in
    }


    @Override
    public void loop() throws InterruptedException {
        //TODO - fill in
    }


    @Override
    public void onSuspend() {
        //TODO - fill in
    }


    @Override
    public void onTerminate() {
        //TODO - fill in
    }


    @Override
    public boolean planSubtrajectory(Trajectory traj, double startDistance, double endDistance) {
        //TODO - fill in
        return false;
    }


    @Override
    public void onReceiveNegotiationRequest(String strategy) {
        //TODO - this will be the subject of a future work item
    }
}
