package gov.dot.fhwa.saxton.carma.guidance.plugins;

/**
 * Mock implementation of something that might resemble a RouteFollowingPlugin
 *
 * Just reports a the name and versionId and toggles its activation status whenever it is activated
 */
public class MockRouteFollowingPlugin extends AbstractMockPlugin {
    public MockRouteFollowingPlugin(PluginServiceLocator pluginServiceLocator) {
        super(pluginServiceLocator);
        name = "MockRouteFollowingPlugin";
        versionId = "v01a";
    }
}
