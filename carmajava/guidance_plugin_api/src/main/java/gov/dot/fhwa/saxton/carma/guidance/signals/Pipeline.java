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

import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.LinkedList;

/**
 * Structure to enable the composition of Filters which, itself, may
 * be treated as an instance of Filter.
 */
public class Pipeline<T> implements Filter<T> {
    private List<Filter<T>> filters = new LinkedList<>();

    /**
     * Default constructor
     */
    public Pipeline (){};

    /**
     * Constructor
     * @param filters a list of filters to applied in the provided order
     */
    public Pipeline (Filter<T>... filters) {
        this.filters = new LinkedList<>(Arrays.asList(filters));
    }

	@Override
	public Optional<Signal<T>> apply(Signal<T> signal) {
        Optional<Signal<T>> res = Optional.of(signal);
        
        // Apply all the filters, continuing if the filter returns a value
		for (Filter<T> filter : filters) {
            if (res.isPresent()) {
                res = filter.apply(res.get());
            } else {
                break; // No need to check the rest of the pipeline anyway
            }
        }

        return res;
    }
    
    /**
     * Add a Filter into the pipeline at the specified position
     * 
     * @param idx The index at which to insert the Filter
     * @param filter The Filter to insert
     */
    public void compose(int idx, Filter<T> filter) {
        filters.add(idx, filter);
    }

    /**
     * Add a Filter into the pipeline at the end.
     * 
     * @param filter the Filter to insert
     */
    public void compose(Filter<T> filter) {
        filters.add(filter);
    }
    
    /**
     * Calls reset() on all filters to removed maintained state but not parameters
     */
    public void reset() {
        for (Filter<T> filter: filters) {
            filter.reset();
        }
    }
}
