/*
 * Copyright (C) 2018-2020 LEIDOS.
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
  protected BlockingQueue<RosServiceResponse<T>> value = new LinkedBlockingQueue<>();

  /**
   * Signal completion of this RosServiceResult. This should wake up any threads waiting on this future.
   */
  public void complete(RosServiceResponse<T> value) {
    try {
      this.value.put(value);
    } catch (InterruptedException e) {
      Thread.currentThread().interrupt();
      throw new RuntimeException("Unable to complete RosServiceResult!");
    }
  }

  /**
   * Sleep while waiting for the value of this future to be available.
   * <p>
   * It is not recommended for more than one thread to wait on a single RosServiceResult instance.
   * The underlying implementation will only supply one value per call of complete(). This may
   * lead to race conditions or deadlocks if more than one thread waits on it.
   * 
   * @throws InterruptedException If the thread is interrupted while waiting for the value
   */
  public RosServiceResponse<T> get() throws InterruptedException {
    return value.take();
  }

  /**
   * Sleep while waiting for the value of this future to be available, timing out after a fixed duration.
   * <p>
   * It is not recommended for more than one thread to wait on a single RosServiceResult instance.
   * The underlying implementation will only supply one value per call of complete(). This may
   * lead to race conditions or deadlocks if more than one thread waits on it.
   * 
   * @return The RosServiceResponse<T> if the value became available before timeout or null if it timed out
   * @throws InterruptedException If the thread is interrupted while waiting for the value
   */
  public RosServiceResponse<T> get(long timeout, TimeUnit unit) throws InterruptedException {
    return value.poll(timeout, unit);
  }
}