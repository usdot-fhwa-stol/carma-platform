package gov.dot.fhwa.saxton.carma.guidance.plugins;

/**
 * Mock implementation of something that might resemble a CrusingPlugin
 *
 * Just reports a the name and versionId and toggles its activation status whenever it is activated
 */
public class MockCruisingPlugin extends AbstractMockPlugin {

    public MockCruisingPlugin(PluginServiceLocator pluginServiceLocator) {
        super(pluginServiceLocator);
        name = "MockCruisingPlugin";
        versionId = "v01a";
    }
}
