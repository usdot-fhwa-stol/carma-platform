package gov.dot.fhwa.saxton.carma.guidance.params;

import org.ros.node.parameter.ParameterTree;

import java.util.List;
import java.util.Map;

/**
 * ROS-backed implementation of ParameterSource interface, delegating to ParameterTree
 */
public class RosParameterSource implements ParameterSource {
    protected ParameterTree params;

    public RosParameterSource(ParameterTree params) {
        this.params = params;
    }

    @Override public boolean getBoolean(String key) {
        return params.getBoolean(key);
    }

    @Override public boolean getBoolean(String key, boolean defaultVal) {
        return params.getBoolean(key, defaultVal);
    }

    @Override public int getInteger(String key) {
        return params.getInteger(key);
    }

    @Override public int getInteger(String key, int defaultVal) {
        return params.getInteger(key, defaultVal);
    }

    @Override public double getDouble(String key) {
        return params.getDouble(key);
    }

    @Override public double getDouble(String key, double defaultVal) {
        return params.getDouble(key, defaultVal);
    }

    @Override public String getString(String key) {
        return params.getString(key);
    }

    @Override public String getString(String key, String defaultVal) {
        return params.getString(key, defaultVal);
    }

    @Override public List<?> getList(String key) {
        return params.getList(key);
    }

    @Override public List<?> getList(String key, List<?> defaultVal) {
        return params.getList(key, defaultVal);
    }

    @Override public Map<?, ?> getMap(String key) {
        return params.getMap(key);
    }

    @Override public Map<?, ?> getMap(String key, Map<?, ?> defaultVal) {
        return params.getMap(key, defaultVal);
    }

    @Override public void set(String key, boolean value) {
        params.set(key, value);
    }

    @Override public void set(String key, int value) {
        params.set(key, value);
    }

    @Override public void set(String key, double value) {
        params.set(key, value);
    }

    @Override public void set(String key, String value) {
        params.set(key, value);
    }

    @Override public void set(String key, List<?> value) {
        params.set(key, value);
    }

    @Override public void set(String key, Map<?, ?> value) {
        params.set(key, value);
    }

    @Override public boolean has(String key) {
        return params.has(key);
    }

    @Override public void delete(String key) {
        params.delete(key);
    }

    @Override public void addParameterListener(String key, final ParameterListener listener) {
        params.addParameterListener(key, new org.ros.node.parameter.ParameterListener() {
            @Override public void onNewValue(Object o) {
                listener.onParameterChanged(o);
            }
        });
    }
}
