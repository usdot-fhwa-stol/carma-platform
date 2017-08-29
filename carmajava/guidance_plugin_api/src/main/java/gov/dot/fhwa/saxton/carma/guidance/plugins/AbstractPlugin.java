package gov.dot.fhwa.saxton.carma.guidance.plugins;

import sun.reflect.generics.reflectiveObjects.NotImplementedException;

public abstract class AbstractPlugin implements IPlugin {
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
        throw new NotImplementedException();
    }

    @Override public void onReceiveNegotiationRequest() {
        throw new NotImplementedException();
    }

    protected String name;
    protected String versionId;
    protected boolean activation = false;
    protected boolean availability = false;
}
