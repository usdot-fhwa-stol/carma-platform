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

package gov.dot.fhwa.saxton.carma.guidance.util;

import java.util.List;

/**
 * Functional interface for receiving notifications only when the list
 * of known intersections changes.
 */
public interface IntersectionListChangeCallback {
    /**
     * Callback to be invoked when the list of intersections known by Guidance changes.
     */
    void onIntersectionListChanged(List<IntersectionData> data);
}