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

package gov.dot.fhwa.saxton.carma.guidance.pubsub;

import gov.dot.fhwa.saxton.carma.rosutils.*;

import org.ros.exception.RemoteException;
import org.ros.internal.node.response.StatusCode;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;

/**
 * Concrete ROS implementation of the logic outlined in {@link IService}
 *
 * Shares a {@link ServiceClient} between its coworkers (other RosService instances for the same
 * topic) and its parent {@link RosServiceChannel}
 * @param <T> Type parameter for the request message type for the service
 * @param <S> Type parameter for the response message type for the service
 */
public class RosService<T, S> implements IService<T, S> {
    protected ServiceClient<T, S> serviceClient;
    protected RosServiceChannel<T, S> parent;
    protected boolean open = true;

    RosService(ServiceClient<T, S> serviceClient, RosServiceChannel<T, S> parent) {
        this.serviceClient = serviceClient;
        this.parent = parent;
    }

    @Override
    public RosServiceResult<S> callAsync(T request) {
        if (open) {
            return RosServiceSynchronizer.callAsync(serviceClient, request);
        } else {
            RosServiceResult<S> out = new RosServiceResult<>();
            out.complete(new RosServiceResponse<S>(
                    new RemoteException(StatusCode.FAILURE, "Service resource has already been closed!")));
            return out;
        }
    }

    @Override
    public void callSync(T request, final OnServiceResponseCallback<S> callback) {
        if (open) {
            try {
                RosServiceSynchronizer.callSync(serviceClient, request, new ServiceResponseListener<S>() {
                    @Override
                    public void onSuccess(S msg) {
                        callback.onSuccess(msg);
                    }

                    @Override
                    public void onFailure(RemoteException e) {
                        callback.onFailure(e);
                    }
                });
            } catch (InterruptedException e) {
                callback.onFailure(
                        new RemoteException(StatusCode.FAILURE, "Interrupted while waiting for service response!"));
            }
        } else {
            callback.onFailure(new RemoteException(StatusCode.FAILURE, "Service resource has already been closed!"));
        }
    }

    @Override
    public T newMessage() {
        return serviceClient.newMessage();
    }

    @Override
    public void close() {
        parent.notifyClientShutdown();
    }
}
