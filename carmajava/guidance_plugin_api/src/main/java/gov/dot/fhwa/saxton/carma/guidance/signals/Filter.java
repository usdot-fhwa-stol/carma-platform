/*
 * Copyright (C) 2017 LEIDOS.
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

package gov.dot.fhwa.saxton.carma.guidance.signals;

import java.util.Optional;

/**
 * Interface for classes which process Signal objects
 * Used to implement time-series transforms of data
 */
public interface Filter<T> {

    /**
     * Apply the filter to the specified signal value.
     * 
     * @param signal The input signal to feed the Filter
     * @return An optional containing the new value, if one was emitted by the filter.
     */
    Optional<Signal<T>> apply(Signal<T> signal);
    
    /**
     * Removes all maintained state from a filter and resets it to default values
     * Note: This DOES NOT reset configured filter parameters
     * 
     * Example: A PID controller would have its integrator reset but not its setpoint
     */
    void reset();
}
