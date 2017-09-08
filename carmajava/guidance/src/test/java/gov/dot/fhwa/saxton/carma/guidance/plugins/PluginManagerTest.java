package gov.dot.fhwa.saxton.carma.guidance.plugins;

import gov.dot.fhwa.saxton.carma.guidance.pubsub.IPubSubService;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import org.ros.node.ConnectedNode;
import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import java.util.ArrayList;
import java.util.List;

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

        @Override public void planTrajectory() {

        }

        @Override public void onReceiveNegotiationRequest() {

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

        @Override public void planTrajectory() {

        }

        @Override public void onReceiveNegotiationRequest() {

        }
    }

    @Before public void setUp() throws Exception {
        psl = mock(PluginServiceLocator.class);
        ConnectedNode node = mock(ConnectedNode.class);
        when(node.getLog()).thenReturn(log);
        pm = new PluginManager(mock(IPubSubService.class), node);
        pluginClasses = new ArrayList<>();
        plugins = new ArrayList<>();
    }

    @After public void tearDown() throws Exception {
    }

    @Test public void discoverPluginsOnClasspath() throws Exception {
        pluginClasses = pm.discoverPluginsOnClasspath();
        for (Class<? extends IPlugin> pClass : pluginClasses) {
            log.info("Discovered " + pClass.getName());
        }
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
  private Log log = LogFactory.getLog(PluginManagerTest.class);
}
