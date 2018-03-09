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

import org.ros.exception.RemoteException;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;

import gov.dot.fhwa.saxton.carma.rosutils.RosServiceResult;
import gov.dot.fhwa.saxton.carma.rosutils.RosServiceSynchronizer;
import javassist.bytecode.analysis.ControlFlow.Block;

import java.util.concurrent.BlockingQueue;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.LinkedBlockingDeque;
import java.util.concurrent.LinkedBlockingQueue;

/**
 * Concrete ROS implementation of the logic outlined in {@link IServiceChannel}
 *
 * Shares access to a {@link ServiceClient} created by an {@link IServiceChannelFactory} between
 * any number of child {@link RosService} instances
 * @param <T> Type parameter for the service request message
 * @param <S> Type parameter for the service response message
 */
public class RosServiceChannel<T, S> implements IServiceChannel<T, S> {
    protected int numOpenChannels = 0;
    protected boolean open = true;
    protected ServiceClient<T, S> serviceClient;
    protected BlockingQueue<ServiceTask> tasks;
    protected Thread workerThread;

    protected class ServiceTask {
        protected T request;
        protected ServiceResponseListener<S> callback;
    }

    protected class ServiceWorker implements Runnable {
		@Override
		public void run() {
            // Spin on the task queue and run the tasks in order, waiting for each response in turn
            while (!Thread.interrupted()) {
                try {
                    ServiceTask task = tasks.take();
                    RosServiceSynchronizer.callSync(serviceClient, task.request, task.callback);
                } catch (InterruptedException e) {
                }
            }
		}
    }

    RosServiceChannel(ServiceClient<T, S> serviceClient) {
        this.serviceClient = serviceClient;
        this.tasks = new LinkedBlockingQueue<>();
        this.workerThread = new Thread(new ServiceWorker());
        workerThread.start();
    }

    /**
     * Get a new IService instance for this topic
     */
    @Override public IService<T, S> getService() {
        numOpenChannels++;

        return new RosService<>(this);
    }

    /**
     * Notice an IService instance being closed. If there are no remaining instances shut down the resource.
     */
    @Override public void notifyClientShutdown() {
        numOpenChannels--;

        if (numOpenChannels <= 0) {
            close();
        }
    }

    /**
     * Return whether or not the underlying resource for this RosPublicationChannel has been shut down
     */
    @Override public boolean isOpen() {
        return open;
    }

    @Override public void close() {
        open = false;
    }

    /**
     * Get the number of extant IService instances that haven't been closed
     */
    public int getNumOpenChannel() {
        return numOpenChannels;
    }

    protected void submitCall(T request, OnServiceResponseCallback<S> callback) {
        CompletableFuture<Void> blocker = new CompletableFuture<>();
        ServiceTask t = new ServiceTask();
        t.request = request;
        t.callback = new ServiceResponseListener<S>() {
			@Override
			public void onFailure(RemoteException arg0) {
                callback.onFailure(arg0);
                blocker.complete(null);
			}
			@Override
			public void onSuccess(S arg0) {
				callback.onSuccess(arg0);
                blocker.complete(null);
			}
        };

		try {
			tasks.put(t);
            blocker.get();
		} catch (InterruptedException e) {
        } catch (ExecutionException e) {
		}
    }
    
    protected T newMessage() {
        return serviceClient.newMessage();
    }
}
