/*
 * TODO: Copyright (C) 2017 LEIDOS.
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

/**
 * NO-OP Implementation of the RouteFollowingPlugin so the Arbitrator has something to access
 */
public class RouteFollowingPlugin extends AbstractPlugin {

  public RouteFollowingPlugin(PluginServiceLocator psl) {
    super(psl);
  }

	@Override
	public void onInitialize() {
    // NO-OP
	}

	@Override
	public void onResume() {
    // NO-OP
	}

	@Override
	public void loop() throws InterruptedException {
    try {
      Thread.sleep(500);
    } catch (InterruptedException ie) {
      Thread.currentThread().interrupt();
    }
	}

	@Override
	public void onSuspend() {
    // NO-OP
	}

	@Override
	public void onTerminate() {
    // NO-OP
  }
}
