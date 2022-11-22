/*
 * Copyright (C) 2018-2019 LEIDOS.
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

package gov.dot.fhwa.saxton.carma.signal_plugin.appcommon;

import gov.dot.fhwa.saxton.carma.guidance.params.ParameterSource;
import gov.dot.fhwa.saxton.carma.guidance.util.RouteService;
import java.util.*;

/**
 * AppConfig using dvi.properties instead of application.properties
 *
 * Provide a mechanism for external configuration via a dvi.properties outside the jar in the current directory.
 *
 * Using a dvi.properties file in the same directory as the jar overrides the default properties set inside the jar.
 * 
 * NOTE: This implementation will replace all occurrences of "." with "/" to make them valid ROS param names
 */
public class GlidepathAppConfig implements IGlidepathAppConfig {
    
    private ParameterSource params;
    private RouteService    routeService;

    public GlidepathAppConfig(ParameterSource params, RouteService routeService) {
        this.params = params;
        this.routeService = routeService;
    }

    public int getIntValue(String property)   {
        return params.getInteger(toROSString(property));
    }
    
    public String getProperty(String name) {
    	return params.getString(toROSString(name));
    }

    /**
     * Returns provided default value if property is not in properties file
     *
     * @param property
     * @param defaultValue
     * @return int
     */
    public int getDefaultIntValue(String property, int defaultValue)   {
        return params.getInteger(toROSString(property), defaultValue);
    }

    private String toROSString(String name) {
        return "~" + name.replace(".", "/");
    }


    public int getPeriodicDelay() {
        return getIntValue("periodicDelay");
    }

    public String getGpsHost() {
        return getProperty("gps.host");
    }

    public int getGpsPort() {
        return getIntValue("gps.port");
    }

    public int getMaximumSpeed(double downtrack) {
        // In legacy ead this was provided by getIntValue("maximumSpeed")
        return (int) (routeService.getSpeedLimitAtLocation(downtrack).getLimit() * Constants.MPS_TO_MPH);
    }

    public boolean getBooleanValue(String property)   {
        return params.getBoolean(toROSString(property));
    }

    public List<Integer> getIntegerList(String name) {
	return (List<Integer>) params.getList(toROSString(name));
    }

    @Override
    public double getDoubleValue(String property) {
        return params.getDouble(toROSString(property));
    }

    @Override
    public double getDoubleDefaultValue(String property, double defaultValue) {
        return params.getDouble(toROSString(property), defaultValue);
    }

    @Override
    public String toString() {
        return "AppConfig [ gps.host:" + getGpsHost() +
                " gps.port:" + getGpsPort() +
                " periodicDelay:" + getPeriodicDelay() +
                " ]";
    }
}
