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

import gov.dot.fhwa.saxton.carma.guidance.arbitrator.TrajectoryPlanningResponse;
import gov.dot.fhwa.saxton.carma.guidance.plugins.AbstractPlugin;
import gov.dot.fhwa.saxton.carma.guidance.plugins.PluginServiceLocator;
import gov.dot.fhwa.saxton.carma.guidance.trajectory.Trajectory;

public class PlatooningPlugin extends AbstractPlugin {

    protected final String PLATOONING_FLAG = "PLATOONING";
    protected PlatooningState state = new StandbyState();
    
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
    }

    @Override
    public void onResume() {
        this.setAvailability(true);
    }

    @Override
    public void loop() throws InterruptedException {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void onSuspend() {
        this.setAvailability(true);
    }

    @Override
    public void onTerminate() {
        // NO-OP
    }

    protected void setState(PlatooningState state) {
        log.info(this.getClass().getSimpleName() + "is changing from " + this.state.toString() + " to " + state.toString());
        this.state = state;
    }
    
    @Override
    public TrajectoryPlanningResponse planTrajectory(Trajectory traj, double expectedEntrySpeed) {
        return this.state.planTrajectory(this, log, pluginServiceLocator, traj, expectedEntrySpeed);
    }
    
    @Override
    public void onReceiveNegotiationRequest(String strategy) {
        this.state.onReceiveNegotiationRequest(this, log, pluginServiceLocator, strategy);
    }
}
