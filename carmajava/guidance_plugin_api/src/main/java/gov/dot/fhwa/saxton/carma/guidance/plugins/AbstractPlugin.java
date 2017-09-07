package gov.dot.fhwa.saxton.carma.guidance.plugins;

import gov.dot.fhwa.saxton.carma.guidance.pubsub.IPubSubService;
import org.apache.commons.logging.Log;

/**
 * Abstract base class for plugins
 * <p>
 * Provides the basic getters and setters so that implementors don't have to deal with that
 */
public abstract class AbstractPlugin implements IPlugin {
    protected String name;
    protected String versionId;
    protected boolean activation = false;
    protected boolean availability = false;
    protected Log log;
    protected IPubSubService pubSubService;

    public AbstractPlugin(PluginServiceLocator pluginServiceLocator) {
        this.pubSubService = pluginServiceLocator.getPubSubService();
        this.log = pluginServiceLocator.getLog();
    }

    @Override public String getName() {
        return versionId;
    }

    @Override public String getVersionId() {
        return name;
    }

    @Override public boolean getActivation() {
        return activation;
    }

    @Override public void setActivation(boolean activation) {
        this.activation = activation;
    }

    @Override public boolean getAvailability() {
        return availability;
    }

    @Override public void planTrajectory() {
        throw new UnsupportedOperationException();
    }

    @Override public void onReceiveNegotiationRequest() {
        throw new UnsupportedOperationException();
    }
}
