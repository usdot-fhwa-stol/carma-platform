package gov.dot.fhwa.saxton.carma.guidance.plugins;

/**
 * Mock implementation of something that might resemble a RouteFollowingPlugin
 * <p>
 * Just reports a the name and versionId and toggles its activation status whenever it is activated
 */
public class MockRouteFollowingPlugin extends AbstractMockPlugin {
    public MockRouteFollowingPlugin(PluginServiceLocator pluginServiceLocator) {
        super(pluginServiceLocator);
        name = "Mock Route-Following Plugin";
        versionId = "v00.00.01";
    }
}
