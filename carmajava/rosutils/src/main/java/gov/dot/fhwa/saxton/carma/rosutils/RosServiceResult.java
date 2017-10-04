/*
 * TODO: Copyright (C) 2017 LEIDOS.
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

package gov.dot.fhwa.saxton.carma.rosutils;

import java.util.concurrent.*;

/**
 * BlockingQueue backed Future<T> styled class for dealing with future tasks that may fail
 * particularly in the context of ROS services
 */
public class RosServiceResult<T> {
  protected BlockingQueue<RosServiceResponse<T>> value = new ArrayBlockingQueue<>(1);

  public void complete(RosServiceResponse<T> value) {
    this.value.add(value);
  }

  public RosServiceResponse<T> get() throws InterruptedException {
    return value.take();
  }

  public RosServiceResponse<T> get(long timeout, TimeUnit unit) throws InterruptedException {
    return value.poll(timeout, unit);
  }
}