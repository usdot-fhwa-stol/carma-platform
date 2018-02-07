/*
 * Copyright (C) 2018 LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

package gov.dot.fhwa.saxton.carma.guidance.params;

import java.util.List;
import java.util.Map;

/**
 * Abstraction layer for org.ros.node.parameter.ParameterTree, will delegate to ParameterTree during
 * run time, but can be mocked or otherwise implemented differently for testing and development.
 */
public interface ParameterSource {

    /**
     * Get a boolean value from the current parameter set
     */
    boolean getBoolean(String key);

    /**
     * Get a boolean value from the current parameter set, returning defaultVal if it is not found
     */
    boolean getBoolean(String key, boolean defaultVal);

    /**
     * Get a integer value from the current parameter set
     */
    int getInteger(String key);

    /**
     * Get a integer value from the current parameter set, returning defaultVal if it is not found
     */
    int getInteger(String key, int defaultVal);

    /**
     * Get a double value from the current parameter set
     */
    double getDouble(String key);

    /**
     * Get a double value from the current parameter set, returning defaultVal if it is not found
     */
    double getDouble(String key, double defaultVal);

    /**
     * Get a String value from the current parameter set
     */
    String getString(String key);

    /**
     * Get a String value from the current parameter set, returning defaultVal if it is not found
     */
    String getString(String key, String defaultVal);

    /**
     * Get a List<?> value from the current parameter set
     */
    List<?> getList(String key);

    /**
     * Get a List<?> value from the current parameter set, returning defaultVal if it is not found
     */
    List<?> getList(String key, List<?> defaultVal);

    /**
     * Get a Map<?, ?> value from the current parameter set
     */
    Map<?, ?> getMap(String key);

    /**
     * Get a Map<?, ?> value from the current parameter set, returning defaultVal if it is not found
     */
    Map<?, ?> getMap(String key, Map<?, ?> defaultVal);

    /**
     * Set a boolean value in the current parameter set, updating it if it already exists
     */
    void set(String key, boolean value);

    /**
     * Set a integer value in the current parameter set, updating it if it already exists
     */
    void set(String key, int value);

    /**
     * Set a double value in the current parameter set, updating it if it already exists
     */
    void set(String key, double value);

    /**
     * Set a String value in the current parameter set, updating it if it already exists
     */
    void set(String key, String value);

    /**
     * Set a List<?></?> value in the current parameter set, updating it if it already exists
     */
    void set(String key, List<?> value);

    /**
     * Set a Map<?, ?> value in the current parameter set, updating it if it already exists
     */
    void set(String key, Map<?, ?> value);

    /**
     * Returns true if the current parameter set has a value for the specified key
     */
    boolean has(String key);

    /**
     * Remote the specified key from the current parameter set
     */
    void delete(String key);

    /**
     * Add a listener to track changes in a parameters value. The ParameterListener's method will
     * be called when the value changes.
     *
     * @param key The parameter to track
     * @param value The ParameterListener to invoke when it changes
     */
    void addParameterListener(String key, ParameterListener value);
}
