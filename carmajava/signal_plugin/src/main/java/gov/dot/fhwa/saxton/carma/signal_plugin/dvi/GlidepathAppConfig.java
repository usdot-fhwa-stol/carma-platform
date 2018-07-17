package gov.dot.fhwa.saxton.carma.signal_plugin.dvi;

import org.springframework.context.annotation.Configuration;
import org.springframework.context.annotation.PropertySource;
import org.springframework.core.env.Environment;

import javax.inject.Inject;
import java.util.Map;

/**
 * AppConfig using dvi.properties instead of application.properties
 *
 * Provide a mechanism for external configuration via a dvi.properties outside the jar in the current directory.
 *
 * Using a dvi.properties file in the same directory as the jar overrides the default properties set inside the jar.
 */
@Configuration
@PropertySource( value = { "classpath:/dvi.properties", "file:dvi.properties" }, ignoreResourceNotFound = true )
public class GlidepathAppConfig implements IGlidepathAppConfig {
    @Inject
    Environment env;
    
    Map<String, String> properties;

    public int getIntValue(String property)   {
        String value = env.getProperty(property);
        return Integer.parseInt(value);
    }
    
    public String getProperty(String name) {
    	return env.getProperty(name);
    }

    /**
     * Returns provided default value if property is not in properties file
     *
     * @param property
     * @param defaultValue
     * @return int
     */
    public int getDefaultIntValue(String property, int defaultValue)   {
        String value = env.getProperty(property);
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
        return env.getProperty("gps.host");
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
        return env.getProperty("xgv.ipaddress");
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
        return env.getProperty("ucr.ipaddress");
    }

    public int getXgvInitTimeout() {
        return getIntValue("xgv.inittimeout");
    }

    public boolean getUcrEnabled() {
        return Boolean.parseBoolean(env.getProperty("ucr.enabled"));
    }

    public double getMaxAccel() {
        return Double.parseDouble(getProperty("maximumAccel")); }

    public String getCanDeviceName() { return env.getProperty("can.candevice"); }

    public String getNativeLib() { return env.getProperty("can.lib"); }

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
        return Double.parseDouble(env.getProperty(property));
    }

    @Override
    public double getDoubleDefaultValue(String property, double defaultValue) {
        String value = env.getProperty(property);

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

    @Override
    public String toString() {
        return "AppConfig [ gps.host:" + getGpsHost() +
                " gps.port:" + getGpsPort() +
                " periodicDelay:" + getPeriodicDelay() +
                " ]";
    }
}