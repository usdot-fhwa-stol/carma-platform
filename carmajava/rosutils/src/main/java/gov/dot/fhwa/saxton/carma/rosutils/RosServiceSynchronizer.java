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

import org.ros.node.service.*;
import org.ros.exception.*;

/**
 * Static class to provide easy facilities for synchronizing ROS service calls
 * <p>
 * Use this class to either produce awaitable futures as a result of ROS service calls or to ensure
 * that the listener is executed synchronously on the same thread as the calling code.
 */
public class RosServiceSynchronizer {

  /**
   * Asynchronously call a ROS service, returning a future that can be waited on as a result
   * 
   * @param client the {@link org.ros.node.service.ServiceClient} instance to use for communication
   * @param request The request message to send
   * @returns An awaitable future in the form of a {@link RosServiceResult}  instance
   */
  public static <R, T> RosServiceResult<T> callAsync(ServiceClient<R, T> client, R request) {
    final RosServiceResult<T> res = new RosServiceResult<>();

    client.call(request, new ServiceResponseListener<T>() {
      @Override
      public void onSuccess(T success) {
        res.complete(new RosServiceResponse<T>(success));
      }

      @Override
      public void onFailure(RemoteException e) {
        res.complete(new RosServiceResponse<T>(e));
      }
    });

    return res;
  }

  /**
   * Synchronously call a ROS service, handling the results with the ServiceResponseListener
   * 
   * @param client the {@link org.ros.node.service.ServiceClient} instance to use for communication
   * @param request The request message to send
   * @param listener The callbacks to execute upon success or failure of the ROS service call
   * @throws InterruptedException if the thread is interrupted while waiting for the call to complete
   */
  public static final <R, T> void callSync(ServiceClient<R, T> client, R request, ServiceResponseListener<T> listener)
      throws InterruptedException {
    final RosServiceResult<T> res = new RosServiceResult<>();

    client.call(request, new ServiceResponseListener<T>() {
      @Override
      public void onSuccess(T success) {
        res.complete(new RosServiceResponse<T>(success));
      }

      @Override
      public void onFailure(RemoteException e) {
        res.complete(new RosServiceResponse<T>(e));
      }
    });

    res.get().get(listener);
  }
}