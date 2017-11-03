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

package gov.dot.fhwa.saxton.carma.guidance.plugins;

import gov.dot.fhwa.saxton.carma.guidance.trajectory.Trajectory;

/**
 * NoOp implementation of IPlugin
 * <p>
 * Does not make any modification to trajectories, does not compute any values,
 * does not take any inputs. Exists to satisfy the minimal arbitrator as a lateral
 * plugin.
 */
public class NoOpPlugin extends AbstractPlugin {
  private static final long SLEEP_DURATION = 10000;

  public NoOpPlugin(PluginServiceLocator psl)  {
    super(psl);
    version.setName("NoOp Plugin");
    version.setMajorRevision(0);
    version.setIntermediateRevision(0);
    version.setMinorRevision(1);
  }

  @Override
  public void onInitialize() {
    // NO OP
  }

  @Override
  public void onResume() {
    // NO OP
  }

  @Override
  public void loop() throws InterruptedException {
    Thread.sleep(SLEEP_DURATION);
  }

  @Override
  public void onSuspend() {
    // NO OP
  }

  @Override
  public void onTerminate() {
    // NO OP
  }

  @Override
  public void planTrajectory(Trajectory traj, double expectedEntrySpeed) {
    // NO OP
  }
}