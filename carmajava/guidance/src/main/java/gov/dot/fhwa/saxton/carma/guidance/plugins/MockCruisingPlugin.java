package gov.dot.fhwa.saxton.carma.guidance.plugins;

import gov.dot.fhwa.saxton.carma.guidance.pubsub.IPubSubService;

public class MockCruisingPlugin extends AbstractMockPlugin {

    public MockCruisingPlugin(IPubSubService pubSubService) {
        super(pubSubService);
        name = "MockCruisingPlugin";
        versionId = "v0.1a";
    }

    @Override protected void computeAvailability() {
        availability = Math.random() > 0.5;
    }
}
