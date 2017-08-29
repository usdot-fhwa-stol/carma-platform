package gov.dot.fhwa.saxton.carma.guidance.plugins;

import gov.dot.fhwa.saxton.carma.guidance.pubsub.IPubSubService;

public class MockRouteFollowingPlugin extends AbstractMockPlugin {
    public MockRouteFollowingPlugin(IPubSubService pubSubService) {
        super(pubSubService);
        name = "MockRouteFollowingPlugin";
        versionId = "v0.1a";
    }
}
