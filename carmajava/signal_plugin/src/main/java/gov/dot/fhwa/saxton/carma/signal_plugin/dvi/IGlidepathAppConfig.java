package gov.dot.fhwa.saxton.carma.signal_plugin.dvi;

import gov.dot.fhwa.saxton.utils.IAppConfig;

/**
 * Glidepath specific appconfig interface
 */
public interface IGlidepathAppConfig extends IAppConfig {
    int getPeriodicDelay();
    int getUiRefresh();
    String getGpsHost();
    int getGpsPort();
    int getJausUdpPort();
    int getSoftwareJausId();
    int getJausRetryLimit();
    String getXgvIpAddress();
    int getXgvNodeId();
    int getXgvSubsystemId();
    int getXgvInstanceId();
    boolean getXgvMPAck();
    boolean getLogRealTimeOutput();
    int getMpdJausId();
    int getVssJausId();
    int getPdJausId();
    int getGpsUdpPort();
    int getMaximumSpeed();
    int getXgvSocketTimeout();
    boolean getAutoStartConsumption();
    int getUcrPort();
    String getUcrIpAddress();
    int getXgvInitTimeout();
    boolean getUcrEnabled();
    double getMaxAccel();
    String getCanDeviceName();
    String getNativeLib();
    double getDefaultSpeed();
}
