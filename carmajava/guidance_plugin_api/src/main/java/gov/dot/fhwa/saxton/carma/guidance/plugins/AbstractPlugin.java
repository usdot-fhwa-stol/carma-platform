package gov.dot.fhwa.saxton.carma.guidance.plugins;

import gov.dot.fhwa.saxton.carma.guidance.pubsub.IPubSubService;
import org.apache.commons.logging.Log;

import java.util.ArrayList;
import java.util.EventListener;
import java.util.List;

/**
 * Abstract base class for plugins
 * <p>
 * Provides the basic getters and setters so that implementors don't have to deal with that
 */
public abstract class AbstractPlugin implements IPlugin {
    protected String name;
    protected String versionId;
    protected Log log;
    protected IPubSubService pubSubService;

    // Private fields so that extendees can't access them
    private boolean activation = false;
    private boolean availability = false;
    private List<AvailabilityListener> availabilityListeners = new ArrayList<>();

    public AbstractPlugin(PluginServiceLocator pluginServiceLocator) {
        this.pubSubService = pluginServiceLocator.getPubSubService();
        this.log = pluginServiceLocator.getLog();
    }

    @Override public String getName() {
        return name;
    }

    @Override public String getVersionId() {
        return versionId;
    }

    @Override public boolean getActivation() {
        return activation;
    }

    @Override public void setActivation(boolean activation) {
        this.activation = activation;
    }

    @Override public final boolean getAvailability() {
        return availability;
    }

    @Override public void planTrajectory() {
        throw new UnsupportedOperationException();
    }

    @Override public void onReceiveNegotiationRequest() {
        throw new UnsupportedOperationException();
    }

    @Override
    public final void registerAvailabilityListener(AvailabilityListener availabilityListener) {
        availabilityListeners.add(availabilityListener);
    }

    /**
     * Use this method to change the plugin's availability status
     *
     * Will automatically ensure all availability listeners are notified
     *
     * @param availability
     */
    protected final void setAvailability(boolean availability) {
        if (this.availability != availability) {
            this.availability = availability;
            log.debug("Availability Listeners " + availabilityListeners.size());
            for (AvailabilityListener el : availabilityListeners) {
                el.onAvailabilityChange(this, availability);
            }
        }
    }
}
