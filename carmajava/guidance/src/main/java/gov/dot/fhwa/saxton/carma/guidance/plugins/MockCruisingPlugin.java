package gov.dot.fhwa.saxton.carma.guidance.plugins;

/**
 * Mock implementation of something that might resemble a CrusingPlugin
 * <p>
 * Just reports a the name and versionId and toggles its activation status whenever it is activated
 */
public class MockCruisingPlugin extends AbstractMockPlugin {

    public MockCruisingPlugin(PluginServiceLocator pluginServiceLocator) {
        super(pluginServiceLocator);
        name = "Mock Cruising Plugin";
        versionId = "v00.00.01";
    }
}
