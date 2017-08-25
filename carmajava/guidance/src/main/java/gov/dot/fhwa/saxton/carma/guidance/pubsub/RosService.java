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

package gov.dot.fhwa.saxton.carma.guidance.pubsub;

import org.ros.exception.RemoteException;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;

public class RosService<T, S> implements IService<T, S> {
    protected ServiceClient<T, S> serviceClient;
    protected RosServiceChannel<T, S> parent;
    protected boolean open = true;

    RosService(ServiceClient<T, S> serviceClient, RosServiceChannel<T, S> parent) {
        this.serviceClient = serviceClient;
        this.parent = parent;
    }

    @Override public void call(T request, final OnServiceResponseCallback<S> callback) {
        if (open) {
            serviceClient.call(request, new ServiceResponseListener<S>() {
                @Override public void onSuccess(S s) {
                    callback.onSuccess(s);
                }

                @Override public void onFailure(RemoteException e) {
                    callback.onFailure(e);
                }
            });
        }
    }

    @Override public void close() {
        parent.notifyClientShutdown();
    }
}
