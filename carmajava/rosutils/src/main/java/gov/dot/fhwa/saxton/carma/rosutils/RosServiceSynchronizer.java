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