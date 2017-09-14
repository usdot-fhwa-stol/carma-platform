package gov.dot.fhwa.saxton.carma.guidance.params;

import java.util.List;
import java.util.Map;

/**
 * Abstraction layer for org.ros.node.parameter.ParameterTree, will delegate to ParameterTree during
 * run time, but can be mocked or otherwise implemented differently for testing and development.
 */
public interface ParameterSource {

    boolean getBoolean(String key);

    boolean getBoolean(String key, boolean defaultVal);

    int getInteger(String key);

    int getInteger(String key, int defaultVal);

    double getDouble(String key);

    double getDouble(String key, double defaultVal);

    String getString(String key);

    String getString(String key, String defaultVal);

    List<?> getList(String key);

    List<?> getList(String key, List<?> defaultVal);

    Map<?, ?> getMap(String key);

    Map<?, ?> getMap(String key, Map<?, ?> defaultVal);

    void set(String key, boolean value);

    void set(String key, int value);

    void set(String key, double value);

    void set(String key, String value);

    void set(String key, List<?> value);

    void set(String key, Map<?, ?> value);

    boolean has(String key);

    void delete(String key);

    void addParameterListener(String key, ParameterListener value);
}
