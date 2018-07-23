package gov.dot.fhwa.saxton.carma.signal_plugin.dvi;

import gov.dot.fhwa.saxton.carma.guidance.params.ParameterSource;

/**
 * AppConfig using dvi.properties instead of application.properties
 *
 * Provide a mechanism for external configuration via a dvi.properties outside the jar in the current directory.
 *
 * Using a dvi.properties file in the same directory as the jar overrides the default properties set inside the jar.
 */
public class GlidepathAppConfig implements IGlidepathAppConfig {
    
    private ParameterSource params;

    public GlidepathAppConfig(ParameterSource params) {
        this.params = params;
    }

    public int getIntValue(String property)   {
        return params.getInteger("~/" + property);
    }
    
    public String getProperty(String name) {
    	return params.getString("~/" + name);
    }

    /**
     * Returns provided default value if property is not in properties file
     *
     * @param property
     * @param defaultValue
     * @return int
     */
    public int getDefaultIntValue(String property, int defaultValue)   {
        return params.getInteger("~/" + property, defaultValue);
    }


    public int getPeriodicDelay() {
        return getIntValue("periodicDelay");
    }

    public int getUiRefresh() {
        return getIntValue("uiRefresh");
    }

    public String getGpsHost() {
        return getProperty("gps.host");
    }

    public int getGpsPort() {
        return getIntValue("gps.port");
    }

    public int getJausUdpPort() {
        return getIntValue("xgv.udpport");
    }

    public int getSoftwareJausId() {
        return getIntValue("xgv.softwarejausid");
    }

    public int getJausRetryLimit() {
        return getIntValue("xgv.retrylimit");
    }

    public String getXgvIpAddress() {
        return getProperty("xgv.ipaddress");
    }

    public int getXgvNodeId() { return getIntValue("xgv.nodeid"); }

    public int getXgvSubsystemId() { return getIntValue("xgv.subsystemid"); }

    public int getXgvInstanceId() { return getIntValue("xgv.instanceid"); }

    public boolean getXgvMPAck() { return getBooleanValue("xgv.motionprofileack"); }

    public boolean getLogRealTimeOutput() { return getBooleanValue("log.stdout"); }

    public int getMpdJausId() {
        return getIntValue("xgv.mpdjausid");
    }

    public int getVssJausId() {
        return getIntValue("xgv.vssjausid");
    }

    public int getPdJausId() {
        return getIntValue("xgv.pdjausid");
    }

    public int getGpsUdpPort() {
        return getIntValue("gps.udpport");
    }

    public int getMaximumSpeed() {
        return getIntValue("maximumSpeed");
    }

    public int getXgvSocketTimeout() { return getIntValue("xgv.timeout"); };

    public boolean getAutoStartConsumption() {
        return getBooleanValue("autoStartConsumption");
    }

    public int getUcrPort() {
        return getIntValue("ucr.port");
    }

    public String getUcrIpAddress() {
        return getProperty("ucr.ipaddress");
    }

    public int getXgvInitTimeout() {
        return getIntValue("xgv.inittimeout");
    }

    public boolean getUcrEnabled() {
        return Boolean.parseBoolean(getProperty("ucr.enabled"));
    }

    public double getMaxAccel() {
        return Double.parseDouble(getProperty("maximumAccel")); }

    public String getCanDeviceName() { return getProperty("can.candevice"); }

    public String getNativeLib() { return getProperty("can.lib"); }

    public double getDefaultSpeed() {
        return Double.parseDouble(getProperty("defaultspeed"));
    }

    public boolean getBooleanValue(String property)   {
        return params.getBoolean("~/" + property);
    }

    @Override
    public double getDoubleValue(String property) {
        return params.getDouble("~/" + property);
    }

    @Override
    public double getDoubleDefaultValue(String property, double defaultValue) {
        return params.getDouble("~/" + property, defaultValue);
    }

    @Override
    public String toString() {
        return "AppConfig [ gps.host:" + getGpsHost() +
                " gps.port:" + getGpsPort() +
                " periodicDelay:" + getPeriodicDelay() +
                " ]";
    }
}