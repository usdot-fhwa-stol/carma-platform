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

package gov.dot.fhwa.saxton.carma.guidance.signals;

/**
 * Basic datum for use with Signals classes
 * 
 * Represents a generic kind of time-variant data for processing by
 * filters.
 */
public class Signal<T> {
    private T data;
    private double timestamp;

    /**
     * Construct a signal holding the data and it's associated timestamp
     * in some arbitrary reference frame
     */
    public Signal(T data, double timestamp) {
        this.data = data;
        this.timestamp = timestamp;
    }

    /**
     * Construct a signal holding the data and it's associated timestamp
     * in ms resolution Unix time, using the current system time.
     */
    public Signal(T data) {
        this.data = data;
        this.timestamp = (double) System.currentTimeMillis();
    }

    /**
     * Get the data associated with this signal
     */
    public T getData() {
        return data;
    }

    /**
     * Get the timestamp associated with this signal
     */
    public double getTimestamp() {
        return timestamp;
    }
}
