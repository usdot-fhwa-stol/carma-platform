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

/**
 * Mock implementation of something that might resemble a RouteFollowingPlugin
 * <p>
 * Just reports a the name and versionId and toggles its activation status whenever it is activated
 */
public class MockRouteFollowingPlugin extends AbstractMockPlugin {
    public MockRouteFollowingPlugin(PluginServiceLocator pluginServiceLocator) {
        super(pluginServiceLocator);
        version.setName("Mock Route-Following Plugin");
        version.setMajorRevision(0);
        version.setIntermediateRevision(0);
        version.setMinorRevision(1);
    }
}
