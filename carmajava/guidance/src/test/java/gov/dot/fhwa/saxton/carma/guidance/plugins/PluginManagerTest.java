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

import gov.dot.fhwa.saxton.carma.guidance.pubsub.IPubSubService;
import gov.dot.fhwa.saxton.carma.guidance.trajectory.Trajectory;
import gov.dot.fhwa.saxton.carma.guidance.util.ILogger;
import gov.dot.fhwa.saxton.carma.guidance.util.ILoggerFactory;
import gov.dot.fhwa.saxton.carma.guidance.util.LoggerManager;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import org.ros.node.ConnectedNode;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;
import gov.dot.fhwa.saxton.carma.guidance.GuidanceState;
import gov.dot.fhwa.saxton.carma.guidance.IGuidanceCommands;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.IManeuverInputs;

import static org.junit.Assert.*;
import static org.mockito.Mockito.*;

public class PluginManagerTest {


    public class TestPlugin1 implements IPlugin {

        @Override public String getName() {
            return null;
        }

        @Override public String getVersionId() {
            return null;
        }

        @Override public void onInitialize() {

        }

        @Override public void onResume() {

        }

        @Override public void loop() throws InterruptedException {

        }

        @Override public void onSuspend() {

        }

        @Override public void onTerminate() {

        }

        @Override public boolean getActivation() {
            return false;
        }

        @Override public void setActivation(boolean activation) {

        }

        @Override public boolean getAvailability() {
            return false;
        }

        @Override public void planTrajectory(Trajectory traj, double expectedEntrySpeed) {

        }

        @Override public void onReceiveNegotiationRequest() {

        }

        @Override
        public void registerAvailabilityListener(AvailabilityListener listener) {

        }
    }

    public class TestPlugin2 implements IPlugin {

        @Override public String getName() {
            return null;
        }

        @Override public String getVersionId() {
            return null;
        }

        @Override public void onInitialize() {

        }

        @Override public void onResume() {

        }

        @Override public void loop() throws InterruptedException {

        }

        @Override public void onSuspend() {

        }

        @Override public void onTerminate() {

        }

        @Override public boolean getActivation() {
            return false;
        }

        @Override public void setActivation(boolean activation) {

        }

        @Override public boolean getAvailability() {
            return false;
        }

        @Override public void planTrajectory(Trajectory traj, double expectedEntrySpeed) {

        }

        @Override public void onReceiveNegotiationRequest() {

        }

        @Override
        public void registerAvailabilityListener(AvailabilityListener listener) {

        }
    }

    @Before public void setUp() throws Exception {
        ILoggerFactory mockFact = mock(ILoggerFactory.class);
        ILogger mockLogger = mock(ILogger.class);
        when(mockFact.createLoggerForClass(anyObject())).thenReturn(mockLogger);
        LoggerManager.setLoggerFactory(mockFact);
        psl = mock(PluginServiceLocator.class);
        ConnectedNode node = mock(ConnectedNode.class);
        pm = new PluginManager(new AtomicReference<GuidanceState>(GuidanceState.DRIVERS_READY), mock(IPubSubService.class), 
        mock(IGuidanceCommands.class), mock(IManeuverInputs.class), node);
        pluginClasses = new ArrayList<>();
        plugins = new ArrayList<>();
    }

    @After public void tearDown() throws Exception {
    }

    @Test public void discoverPluginsOnClasspath() throws Exception {
        pluginClasses = pm.discoverPluginsOnClasspath();
        assertTrue(pluginClasses.contains(TestPlugin1.class));
        assertTrue(pluginClasses.contains(TestPlugin2.class));
    }

    @Test public void instantiatePluginsFromClasses() throws Exception {
      List<Class<? extends IPlugin>> pluginList = new ArrayList<>();
      pluginList.add(MockRouteFollowingPlugin.class);
      pluginList.add(MockCruisingPlugin.class);

      List<IPlugin> instances = pm.instantiatePluginsFromClasses(pluginList, psl);

      assertEquals(2, instances.size());

      boolean foundMockCruisingPlugin = false;
      boolean foundMockRouteFollowingPlugin = false;

      for (IPlugin p : instances) {
        if (p instanceof MockRouteFollowingPlugin) {
          foundMockRouteFollowingPlugin = true;
        }
        if (p instanceof MockCruisingPlugin) {
          foundMockCruisingPlugin = true;
        }
      }

      assertTrue(foundMockCruisingPlugin);
      assertTrue(foundMockRouteFollowingPlugin);
    }

    private PluginServiceLocator psl;
    private PluginManager pm;
    private List<Class<? extends IPlugin>> pluginClasses;
    private List<IPlugin> plugins;
}
