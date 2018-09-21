package gov.dot.fhwa.saxton.carma.guidance.util;

import java.util.List;

/**
 * Interface to be implemented by backend Guidance software component
 * <p>
 * Provides V2I data to IPlugin instances via the PluginServiceLocator
 */
public interface V2IService {
    void registerV2IDataCallback(V2IDataCallback callback);
    List<IntersectionData> getV2IData();
}