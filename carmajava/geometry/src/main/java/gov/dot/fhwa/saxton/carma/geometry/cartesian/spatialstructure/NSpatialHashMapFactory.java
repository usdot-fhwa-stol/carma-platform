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

package gov.dot.fhwa.saxton.carma.geometry.cartesian.spatialstructure;

import java.util.HashMap;

import gov.dot.fhwa.saxton.carma.geometry.cartesian.AxisAlignedBoundingBox;

/**
 * Factory/builder pattern class for the construction of NSpatialHashMap instances.
 * <p>
 * Assigns the strategies needed by the NSpatialHashMap instance.
 */
public class NSpatialHashMapFactory {
    /**
     * Build a NSpatialHashMap with the default configuration.
     * <p>
     * {@link SimpleHashStrategy} is used to hash objects
     * {@link AxisAlignedBoundingBox} is used for collision detection
     * {@link HashMap} is used for the underlying data structure
     * 
     * The returned map is not thread safe on its own
     */
    public static NSpatialHashMap buildSpatialHashMap(double[] cellDims) {
        return new NSpatialHashMap(new AxisAlignedBoundingBox(), new SimpleHashStrategy(cellDims), new HashMap<>());
    }
}