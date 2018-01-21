/*
 * Copyright (C) 2017 LEIDOS.
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

package gov.dot.fhwa.saxton.carma.plugins.platooning;

import java.util.LinkedList;
import java.util.Queue;
import java.util.SortedSet;
import java.util.TreeSet;

import gov.dot.fhwa.saxton.carma.guidance.arbitrator.TrajectoryPlanningResponse;
import gov.dot.fhwa.saxton.carma.guidance.plugins.AbstractPlugin;
import gov.dot.fhwa.saxton.carma.guidance.plugins.IStrategicPlugin;
import gov.dot.fhwa.saxton.carma.guidance.plugins.PluginServiceLocator;
import gov.dot.fhwa.saxton.carma.guidance.trajectory.Trajectory;

public class PlatooningPlugin extends AbstractPlugin implements IStrategicPlugin {

    protected final String PLATOONING_FLAG = "PLATOONING";
    
    protected IPlatooningState state;
    protected PlatoonManager manager = new PlatoonManager(this);
    protected SortedSet<PlatoonMember> platoon = new TreeSet<>();
    protected Queue<String> statusQueue = new LinkedList<>(); 
    protected double maxAccel = 2.5;
    protected long timestep = 100;
    protected double minimumManeuverLength = 5.0;
    protected long commandTimeout = 3000;
    
    protected CommandGenerator commandGenerator = null;
    protected Thread commandGeneratorThread = null;
    
    public PlatooningPlugin(PluginServiceLocator pluginServiceLocator) {
        super(pluginServiceLocator);
        version.setName("CACC Platooning Plugin");
        version.setMajorRevision(0);
        version.setIntermediateRevision(0);
        version.setMinorRevision(1);
    }

    @Override
    public void onInitialize() {
        log.info("CACC platooning pulgin is initializing...");
        platoon = new TreeSet<>((a, b) -> Double.compare(a.getMemberId(), b.getMemberId()));
        maxAccel = pluginServiceLocator.getParameterSource().getDouble("~platooning_max_accel", 2.5);
        timestep = (long) pluginServiceLocator.getParameterSource().getInteger("~platooning_command_timestep", 100);
        minimumManeuverLength = pluginServiceLocator.getParameterSource().getDouble("~platooning_min_maneuver_length", 5.0);
        commandTimeout = pluginServiceLocator.getParameterSource().getInteger("~platooning_command_timeout", 3000);
    }

    @Override
    public void onResume() {
        if(commandGenerator == null && commandGeneratorThread == null) {
            commandGenerator = new CommandGenerator(this);
            commandGeneratorThread = new Thread(commandGenerator);
            commandGeneratorThread.setName("Platooning Command Generator");
            commandGeneratorThread.start();
        }
        this.setAvailability(true);
    }

    @Override
    public void loop() throws InterruptedException {
        // TODO add logic to handle STATUS messages in statusQueue
        Thread.sleep(10000);
    }

    @Override
    public void onSuspend() {
        this.setAvailability(false);
        if(commandGenerator != null && commandGeneratorThread != null) {
            commandGeneratorThread.interrupt();
            commandGenerator = null;
            commandGeneratorThread = null;
        }
    }

    @Override
    public void onTerminate() {
        // NO-OP
    }

    protected void setState(IPlatooningState state) {
        log.info("Change from " + this.state.toString() + " to " + state.toString());
        this.state = state;
    }
    
    @Override
    public TrajectoryPlanningResponse planTrajectory(Trajectory traj, double expectedEntrySpeed) {
        log.info("Plan Trajectory from " + traj.getStartLocation() + " to " + traj.getEndLocation() + " in state " + state.toString());
        return this.state.planTrajectory(traj, expectedEntrySpeed);
    }
    
    // TODO change the input type from String to maybe a plan object
    /******* this has been removed from the interface...probably needs to be removed from here in lieu of the new negotiation receiver plugin
    @Override
    public void onReceiveNegotiationRequest(String message) {
        log.info("Receive negotiation message: " + message);
        if(message == null) {
            return;
        }
        if(message.startsWith(PlatooningRequests.UPDATE.toString())) {
            statusQueue.add(message);
        } else {
            this.state.onReceiveNegotiationRequest(message);
        }
    }
    *****/

    public double getMaxAccel() {
        return maxAccel;
    }

    public long getTimestep() {
        return timestep;
    }

    public double getMinimumManeuverLength() {
        return minimumManeuverLength;
    }

    public long getCommandTimeout() {
        return commandTimeout;
    }

}
