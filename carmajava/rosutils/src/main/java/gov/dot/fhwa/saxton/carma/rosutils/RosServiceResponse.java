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

import org.ros.node.service.ServiceResponseListener;
import org.ros.exception.RemoteException;

/**
 * Value holder for ROS service response data and/or an error
 */
public class RosServiceResponse<T> {
  protected T value;
  protected boolean valuePresent = false;
  protected RemoteException error;

  public RosServiceResponse(T value) {
    this.value = value;
    valuePresent = true;
  }

  public RosServiceResponse(RemoteException error) {
    this.error = error;
  }

  /**
  * If the call returns correctly the get method will return the value of the response object. If
  * the call failed the get method will re-throw the error captured during communications.
   */
  public T get() throws RemoteException {
    if (valuePresent) {
      return value;
    } else {
      throw error;
    }
  }

  /**
   * Applies the {@link ServiceResponseListener} parameter to the contained service response. If the
   * call was a success, the .onSuccess branch will be executed. If not the .onFailure branch will be
   * executed.
   */
  public void get(ServiceResponseListener<T> listener) {
    if (valuePresent) {
      listener.onSuccess(value);
    } else {
      listener.onFailure(error);
    }
  }
}