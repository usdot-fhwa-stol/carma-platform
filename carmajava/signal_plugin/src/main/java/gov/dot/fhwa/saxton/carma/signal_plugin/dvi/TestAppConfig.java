package gov.dot.fhwa.saxton.carma.signal_plugin.dvi;

import gov.dot.fhwa.saxton.carma.signal_plugin.dvi.IGlidepathAppConfig;

import java.util.HashMap;
import java.util.Map;

/**
 * AppConfig override for unit testing purposes.
 */
public class TestAppConfig implements IGlidepathAppConfig {

    private Map<String, String> properties = new HashMap<>();

    public int getIntValue(String property)   {
        return Integer.parseInt(properties.get(property));
    }

    public String getProperty(String name) {
        return properties.get(name);
    }

    /**
     * Returns provided default value if property is not in properties file
     *
     * @param property
     * @param defaultValue
     * @return int
     */
    public int getDefaultIntValue(String property, int defaultValue)   {
        String value = properties.get(property);
        if (value == null)   {
            return defaultValue;
        }

        int result = 0;
        try   {
            result = Integer.parseInt(value);
        }
        catch(Exception e)   {}

        return result;
    }


    public int getPeriodicDelay() {
        return getIntValue("periodicDelay");
    }

    public int getUiRefresh() {
        return getIntValue("uiRefresh");
    }

    public String getGpsHost() {
        return properties.get("gps.host");
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
        return properties.get("xgv.ipaddress");
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
        return properties.get("ucr.ipaddress");
    }

    public int getXgvInitTimeout() {
        return getIntValue("xgv.inittimeout");
    }

    public boolean getUcrEnabled() {
        return Boolean.parseBoolean(properties.get("ucr.enabled"));
    }

    public double getMaxAccel() {
        return Double.parseDouble(getProperty("maximumAccel")); }

    public String getCanDeviceName() { return properties.get("can.candevice"); }

    public String getNativeLib() { return properties.get("can.lib"); }

    public double getDefaultSpeed() {
        return Double.parseDouble(getProperty("defaultspeed"));
    }

    public boolean getBooleanValue(String property)   {
        boolean bValue = false;

        String value = getProperty(property);
        if (value == null || value.isEmpty())  {
            bValue = false;
        }
        else   {
            bValue = Boolean.parseBoolean(value);
        }

        return bValue;
    }

    @Override
    public double getDoubleValue(String property) {
        return Double.parseDouble(properties.get(property));
    }

    @Override
    public double getDoubleDefaultValue(String property, double defaultValue) {
        String value = properties.get(property);

        if (value == null) {
            return defaultValue;
        } else {
            try {
                return Double.parseDouble(value);
            } catch (NumberFormatException nfe) {
                return defaultValue;
            }
        }
    }

    /**
     * Set the value of a property in this TestAppConfig which can be retrieved by other method calls.
     * Int, boolean, and double type parameters are parsed out of strings when they are retrieved.
     *
     * @param key The name of the property to set
     * @param value The string representation of the value of the property
     */
    public void setProperty(String key, String value) {
        properties.put(key, value);
    }
}
