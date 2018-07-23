package gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.utils;

import gov.dot.fhwa.saxton.carma.signal_plugin.dvi.GlidepathAppConfig;
import gov.dot.fhwa.saxton.carma.signal_plugin.dvi.IGlidepathAppConfig;

/**
 * Wrapper to always return a reference to the Spring Application Context from
 * within non-Spring enabled beans. Unlike Spring MVC's WebApplicationContextUtils
 * we do not need a reference to the Servlet context for this. All we need is
 * for this bean to be initialized during application startup.
 */
public class GlidepathApplicationContext {

    private static IGlidepathAppConfig appConfigOverride = null;

    private GlidepathApplicationContext() {
    }

    private static class GlidepathApplicationContextHolder  {
        private static final GlidepathApplicationContext _instance = new GlidepathApplicationContext();
    }

    public static GlidepathApplicationContext getInstance()
    {
        return GlidepathApplicationContextHolder._instance;
    }

    /**
     * Directly acquire AppConfig, preferring the overridden version if it exists.
     *
     * @return
     */
    public IGlidepathAppConfig getAppConfig()   {
        return appConfigOverride;
    }

    /**
     * Override the current appconfig instance with a new one.
     * @param override The {@link GlidepathAppConfig} instance to be used from now on.
     */
    public void setAppConfigOverride(IGlidepathAppConfig override) {
        appConfigOverride = override;
    }
}